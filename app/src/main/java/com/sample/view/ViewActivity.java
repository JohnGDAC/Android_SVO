/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.sample.view;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.os.Bundle;
import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import android.util.Log;
import android.view.Display;
import android.view.Gravity;
import android.view.Surface;
import android.view.TextureView;
import android.view.SurfaceView;
import android.view.SurfaceHolder;
import android.view.View;
import android.util.Size;
import android.widget.Button;
import android.widget.FrameLayout;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.FileInputStream;

import org.json.JSONException;
import org.json.JSONObject;

import static android.hardware.camera2.CameraMetadata.LENS_FACING_BACK;

public class ViewActivity extends Activity
        implements TextureView.SurfaceTextureListener,
        ActivityCompat.OnRequestPermissionsResultCallback {

    private TextureView textureView_;
    private SurfaceView sufaceView_ = null;
    private Button button_ = null;
    Surface surface_ = null;
    private Size cameraPreviewSize_;
    double[] cal_intrinsic_;
    private boolean is_load_cal_ = false;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        onWindowFocusChanged(true);
        setContentView(R.layout.activity_main);

        button_ = findViewById(R.id.button);
        button_.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(!is_load_cal_) {
                    Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
                    //intent.setType(“image/*”);//选择图片
                    //intent.setType(“audio/*”); //选择音频
                    //intent.setType(“video/*”); //选择视频 （mp4 3gp 是android支持的视频格式）
                    //intent.setType(“video/*;image/*”);//同时选择视频和图片
                    intent.setType("*/*");//无类型限制
                    intent.addCategory(Intent.CATEGORY_OPENABLE);
                    startActivityForResult(intent, 1);
                    return;
                }
                startORsavePic(cal_intrinsic_);
                button_.setEnabled(false);
            }
        });

        sufaceView_ = (SurfaceView)findViewById(R.id.surfaceview);
        sufaceView_.getHolder().addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
                setGLSurface(holder.getSurface());
                startGLRender();
            }
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                createGLRender(getAssets());
                setGLSurface(holder.getSurface());
                startGLRender();
            }
            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                stopGLRender();
                destroyGLRender();
            }
        });

        if (isCamera2Device()) {
            RequestCamera();
        } else {
            Log.e("CameraSample", "Found legacy camera device, this sample needs camera2 device");
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == 1 && resultCode == RESULT_OK) {
            // 处理单个文件URI
            if (data.getData() != null) {
                Uri uri = data.getData();

                StringBuffer json = new StringBuffer();
                InputStreamReader reader;
                try {
                    InputStream fip = getContentResolver().openInputStream(uri);
                    reader = new InputStreamReader(fip);
                }
                catch (FileNotFoundException e) {
                    e.printStackTrace();
                    return;
                }

                try {
                    while (reader.ready()) {
                        json.append((char) reader.read());
                        // 转成char加到StringBuffer对象中
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                    return;
                }

                cal_intrinsic_ = new double[11];
                try {
                    JSONObject jsonObject1 = new JSONObject(json.toString());
                    cal_intrinsic_[0] = jsonObject1.getDouble("w");
                    cal_intrinsic_[1] = jsonObject1.getDouble("h");
                    cal_intrinsic_[2] = jsonObject1.getDouble("fx");
                    cal_intrinsic_[3] = jsonObject1.getDouble("fy");
                    cal_intrinsic_[4] = jsonObject1.getDouble("cx");
                    cal_intrinsic_[5] = jsonObject1.getDouble("cy");
                    cal_intrinsic_[6] = jsonObject1.getDouble("k1");
                    cal_intrinsic_[7] = jsonObject1.getDouble("k2");
                    cal_intrinsic_[8] = jsonObject1.getDouble("p1");
                    cal_intrinsic_[9] = jsonObject1.getDouble("p2");
                    cal_intrinsic_[10] = jsonObject1.getDouble("k3");
                } catch (JSONException e) {
                    e.printStackTrace();
                    return;
                }

                is_load_cal_ = true;
                button_.setText("start");
            }
        }
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            getWindow().getDecorView().setSystemUiVisibility(
                    View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                            | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                            | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                            | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                            | View.SYSTEM_UI_FLAG_FULLSCREEN
                            | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
        }
    }

    private boolean isCamera2Device() {
        CameraManager camMgr = (CameraManager)getSystemService(Context.CAMERA_SERVICE);
        boolean camera2Dev = true;
        try {
            String[] cameraIds = camMgr.getCameraIdList();
            if (cameraIds.length != 0 ) {
                for (String id : cameraIds) {
                    CameraCharacteristics characteristics = camMgr.getCameraCharacteristics(id);
                    int deviceLevel = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL);
                    int facing = characteristics.get(CameraCharacteristics.LENS_FACING);
                    if (deviceLevel == CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY &&
                            facing == LENS_FACING_BACK) {
                        camera2Dev =  false;
                    }
                }
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
            camera2Dev = false;
        }
        return camera2Dev;
    }

    private void createTextureView() {
        textureView_ = (TextureView) findViewById(R.id.texturePreview);
        textureView_.setSurfaceTextureListener(this);
        if (textureView_.isAvailable()) {
            onSurfaceTextureAvailable(textureView_.getSurfaceTexture(),
                    textureView_.getWidth(), textureView_.getHeight());
        }
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        createNativeCamera();

        String log = String.format("textureView size : %d x %d", width, height);
        Log.i("CameraSample", log);

        //resizeTextureView(width, height);
        surface.setDefaultBufferSize(cameraPreviewSize_.getWidth(),
                cameraPreviewSize_.getHeight());

        surface_ = new Surface(surface);
        onPreviewSurfaceCreated(surface_);
    }

    private void resizeTextureView(int textureWidth, int textureHeight) {
        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        int newWidth = textureWidth;
        int newHeight = textureWidth * cameraPreviewSize_.getWidth() / cameraPreviewSize_.getHeight();

        if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
            newHeight = (textureWidth * cameraPreviewSize_.getHeight()) / cameraPreviewSize_.getWidth();
        }
        textureView_.setLayoutParams(
                new FrameLayout.LayoutParams(newWidth, newHeight, Gravity.CENTER));
        String log = String.format("resizeTextureView : %d x %d", newWidth, newHeight);

        Log.i("CameraSample", log);
        configureTransform(newWidth, newHeight);
    }

    /**
     * configureTransform()
     * Courtesy to https://github.com/google/cameraview/blob/master/library/src/main/api14/com/google/android/cameraview/TextureViewPreview.java#L108
     *
     * @param width  TextureView width
     * @param height is TextureView height
     */
    void configureTransform(int width, int height) {
        int mDisplayOrientation = getWindowManager().getDefaultDisplay().getRotation() * 90;
        Matrix matrix = new Matrix();
        if (mDisplayOrientation % 180 == 90) {
            //final int width = getWidth();
            //final int height = getHeight();
            // Rotate the camera preview when the screen is landscape.
            matrix.setPolyToPoly(
                    new float[]{
                            0.f, 0.f, // top left
                            width, 0.f, // top right
                            0.f, height, // bottom left
                            width, height, // bottom right
                    }, 0,
                    mDisplayOrientation == 90 ?
                            // Clockwise
                            new float[]{
                                    0.f, height, // top left
                                    0.f, 0.f,    // top right
                                    width, height, // bottom left
                                    width, 0.f, // bottom right
                            } : // mDisplayOrientation == 270
                            // Counter-clockwise
                            new float[]{
                                    width, 0.f, // top left
                                    width, height, // top right
                                    0.f, 0.f, // bottom left
                                    0.f, height, // bottom right
                            }, 0,
                    4);
        } else if (mDisplayOrientation == 180) {
            matrix.postRotate(180, width / 2, height / 2);
        }
        textureView_.setTransform(matrix);
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface,
                                            int width, int height) {
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        onPreviewSurfaceDestroyed(surface_);
        deleteCamera();
        surface_ = null;
        return true;
    }

    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
    }

    private static final int PERMISSION_REQUEST_CODE_CAMERA = 1;

    public void RequestCamera() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) !=
                PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.CAMERA},
                    PERMISSION_REQUEST_CODE_CAMERA);
            return;
        }
        createTextureView();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        /*
         * if any permission failed, the sample could not play
         */
        if (PERMISSION_REQUEST_CODE_CAMERA != requestCode) {
            super.onRequestPermissionsResult(requestCode,
                    permissions,
                    grantResults);
            return;
        }

        if (grantResults.length == 1 &&
            grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            Thread initCamera = new Thread(new Runnable() {
                public void run() {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            createTextureView();
                        }
                    });
                }
            });
            initCamera.start();
        }
    }

    private void createNativeCamera() {
        Display display = getWindowManager().getDefaultDisplay();
        int height = display.getMode().getPhysicalHeight();
        int width = display.getMode().getPhysicalWidth();

        createCamera(width, height);

        cameraPreviewSize_ = getMinimumCompatiblePreviewSize();

    }

    /*
     * Functions calling into NDKCamera side to:
     *     CreateCamera / DeleteCamera object
     *     Start/Stop Preview
     *     Pulling Camera Parameters
     */
    private native void createCamera(int width, int height);

    private native Size getMinimumCompatiblePreviewSize();

    private native void onPreviewSurfaceCreated(Surface surface);

    private native void onPreviewSurfaceDestroyed(Surface surface);

    private native void deleteCamera();

    private native void startORsavePic(double[] cal_intrinsic);

    private native void createGLRender(AssetManager assetManager);
    private native void destroyGLRender();
    private native void startGLRender();
    private native void stopGLRender();
    private native void setGLSurface(Surface surface);

    static {
        System.loadLibrary("system_main");
    }

}
