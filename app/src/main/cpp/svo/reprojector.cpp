// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <algorithm>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <svo/reprojector.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo/abstract_camera.h>
#include <svo/math_utils.h>

namespace svo {

Reprojector::Reprojector(vk::AbstractCamera* cam, Map& map) :
    map_(map)
{
  initializeGrid(cam);
}

Reprojector::~Reprojector()
{
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
}

void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
  grid_.cell_size = Config::gridSize();
  grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);
  grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);
  grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
  grid_.cell_order.resize(grid_.cells.size());
  for(size_t i=0; i<grid_.cells.size(); ++i)
    grid_.cell_order[i] = i;
  //random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration!
}

void Reprojector::resetGrid()
{
  n_matches_ = 0;
  n_trials_ = 0;
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

/**
 * frame-to-map匹配，计算3d点在当前帧中的像素坐标，注意是优化投影像素坐标，而不是位姿（位姿是固定的）
 * 1、在关键帧map中提取与当前帧有共视点的帧，按距离从小到大排序
 * 2、从前N帧共视帧中投影3d点到当前帧，存入网格，另外将收敛的种子点也投影到网格
 * 3、对每一个网格，从中选一个最好的点，对于该点在观测帧集合中选择与当前帧视差最小的一帧，进行图像配准，优化点在当前帧中的投影像素坐标
 * 4、添加特征点到当前frame，总数限制
 * @param frame       当前帧，只有图像金字塔和初始位姿（与前一帧align得到的）
 * @param overlap_kfs 记录 <共视帧，该共视帧与当前帧的共视点数量>
*/
void Reprojector::reprojectMap(
    FramePtr frame,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
  resetGrid();

  // Identify those Keyframes which share a common field of view.
  // 在关键帧map中提取与当前帧有共视点的帧，计算距离
  SVO_START_TIMER("reproject_kfs");
    std::list< std::pair<FramePtr,double> > close_kfs;
  map_.getCloseKeyframes(frame, close_kfs);

  // Sort KFs with overlap according to their closeness
  // 按距离从小到大排序
  close_kfs.sort([](const std::pair<FramePtr, double>& l, const std::pair<FramePtr, double>& r)
                 {return l.second < r.second;});

  // Reproject all mappoints of the closest N kfs with overlap. We only store
  // in which grid cell the points fall.
  size_t n = 0;
  overlap_kfs.reserve(options_.max_n_kfs);
  // 遍历最近的N帧
  for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
      it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
  {
    FramePtr ref_frame = it_frame->first;
    // 记录 <共视帧，该共视帧与当前帧的共视点数量>
    overlap_kfs.push_back(std::pair<FramePtr,size_t>(ref_frame,0));

    // Try to reproject each mappoint that the other KF observes
    // 遍历3d点（已经三角化过的特征点）
    for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
        it_ftr!=ite_ftr; ++it_ftr)
    {
      // check if the feature has a mappoint assigned
      if((*it_ftr)->point == NULL)
        continue;

      // make sure we project a point only once
      // 设置特征点的投影帧，只投影到某一帧上
      if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
        continue;
      (*it_ftr)->point->last_projected_kf_id_ = frame->id_;
      // 重投影世界点到frame下
      if(reprojectPoint(frame, (*it_ftr)->point))
        overlap_kfs.back().second++;
    }
  }
  SVO_STOP_TIMER("reproject_kfs");

  // Now project all point candidates
  // 重投影种子点，从map_中删除多次没有被观测到的种子点
  SVO_START_TIMER("reproject_candidates");
  {
    // candidates_是收敛的种子点集合 todo
    std::unique_lock<std::mutex> lock(map_.point_candidates_.mut_);
    auto it=map_.point_candidates_.candidates_.begin();
    while(it!=map_.point_candidates_.candidates_.end())
    {
      // 当前帧看不到，一定次数后从种子点集合中删除它
      if(!reprojectPoint(frame, it->first))
      {
        // 后面有地方+1
        it->first->n_failed_reproj_ += 3;
        if(it->first->n_failed_reproj_ > 30)
        {
          map_.point_candidates_.deleteCandidate(*it);
          it = map_.point_candidates_.candidates_.erase(it);
          continue;
        }
      }
      ++it;
    }
  } // unlock the mutex when out of scope
  SVO_STOP_TIMER("reproject_candidates");

  // Now we go through each grid cell and select one point to match.
  // At the end, we should have at maximum one reprojected point per cell.
  SVO_START_TIMER("feature_align");
  // 遍历网格
  for(size_t i=0; i<grid_.cells.size(); ++i)
  {
    // we prefer good quality points over unkown quality (more likely to match)
    // and unknown quality over candidates (position not optimized)
    /**
     * 从网格中选一个最好的点，对于该点在观测帧集合中选择与当前帧视差最小的一帧，进行图像配准，优化点在当前帧中的投影像素坐标
     * 添加特征点到当前frame
    */
    if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
      ++n_matches_;

    // 特征点总数限制
    if(n_matches_ > (size_t) Config::maxFts())
      break;
  }
  SVO_STOP_TIMER("feature_align");
}

bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

/**
 * 从网格中选一个最好的点，对于该点在观测帧集合中选择与当前帧视差最小的一帧，进行图像配准，优化点在当前帧中的投影像素坐标
 * 添加特征点到当前frame
*/
bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
  // 按照特征点的好坏排序，最好是good，默认是unknow，delete是删掉的点
  cell.sort(&Reprojector::pointQualityComparator);
  Cell::iterator it=cell.begin();
  while(it!=cell.end())
  {
    ++n_trials_;

    if(it->pt->type_ == Point::TYPE_DELETED)
    {
      it = cell.erase(it);
      continue;
    }

    // inverse compositional Lucas-Kanade algorithm
    // 图像配准，优化3d点在当前帧中的投影像素坐标。参考帧为3d点观测帧中与当前帧视差最小的一帧
    bool found_match = true;
    if(options_.find_match_direct)
      found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);

    if(!found_match)
    {
      it->pt->n_failed_reproj_++;
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map_.safeDeletePoint(it->pt);
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        map_.point_candidates_.deleteCandidatePoint(it->pt);
      it = cell.erase(it);
      continue;
    }

    // 3d点看到的帧比较多，设为good点
    it->pt->n_succeeded_reproj_++;
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;

    // 添加特征点到当前帧
    Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
    frame->addFeature(new_feature);

    // Here we add a reference in the feature to the 3D point, the other way
    // round is only done if this frame is selected as keyframe.
    new_feature->point = it->pt;

    if(matcher_.ref_ftr_->type == Feature::EDGELET)
    {
      new_feature->type = Feature::EDGELET;
      new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
      new_feature->grad.normalize();
    }

    // If the keyframe is selected and we reproject the rest, we don't have to
    // check this point anymore.
    it = cell.erase(it);

    // Maximum one point per cell.
    return true;
  }
  return false;
}

/**
 * 重投影世界点到frame下
*/
bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
  // 世界点投影到frame下，要求至少有8x8的patch空间
  Vector2d px(frame->w2c(point->pos_));
  if(frame->cam_->isInFrame(px.cast<int>(), 8)) // 8px is the patch size in the matcher
  {
    const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols
                + static_cast<int>(px[0]/grid_.cell_size);
    // 保存到网格 <世界坐标，投影像素坐标>
    grid_.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  return false;
}

} // namespace svo
