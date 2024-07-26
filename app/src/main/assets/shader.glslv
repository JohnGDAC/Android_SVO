// attribute float vPosition;
// attribute float vSensorValue;

attribute vec4 vPoints;
uniform mat4 vViewPose;

attribute vec4 uFragColor;
varying vec4 v_Color;

void main() {
    v_Color = uFragColor;
    //gl_Position = vec4(vSensorValue/9.81, -vPosition, 0, 1); //rotate 90
    gl_Position = vViewPose * vPoints;
    gl_Position = vec4(-gl_Position[1] ,
                        -gl_Position[0],
                         gl_Position[2], 1);  //rotate 90
}
