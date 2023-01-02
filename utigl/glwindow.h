#ifndef UTIGL_GLWINDOW_H
#define UTIGL_GLWINDOW_H

#include <stdio.h>
#include <GL/glut.h>
#include <GL/freeglut_ext.h>
#include <GL/gl.h>
#include <nmath/vec3.h>
#include <nmath/quaternion.h>

#ifndef w_glutSetWindowData
#define w_glutSetWindowData glutSetWindowData
#endif

#ifndef w_glutGetWindowData
#define w_glutGetWindowData glutGetWindowData
#endif

#ifndef w_glutLeaveMainLoop
#define w_glutLeaveMainLoop glutLeaveMainLoop
#endif

#ifndef w_glutCloseFunc
#define w_glutCloseFunc glutCloseFunc
#endif

class CameraCommon {
 public:

  CameraCommon() {
    nearPlane =  0.1;
    farPlane = 100.0;

    cquat = nacb::Quaternion();
    cpos  = nacb::Vec3d(0,0,10);
    fov = 45.0;

    ortho = false;
    object = true;
  }

  virtual ~CameraCommon(){ }
  
  virtual void applyProjection(){
    if(ortho){
      glOrtho(-aspect(), aspect(), -1.0, 1.0, nearPlane, farPlane);
    }
    else 
      gluPerspective(fov, aspect(), nearPlane, farPlane);
  }

  virtual void applyModelview(){
    cquat.conj().glRotate();
    glTranslatef(-cpos.x, -cpos.y, -cpos.z);
  }

  virtual double aspect(){
    return double(width())/double(height());
  }

  virtual int width() = 0;
  virtual int height() = 0;

  bool             object;
  double        nearPlane;
  double         farPlane;
  bool              ortho;
  double              fov;
  nacb::Vec3d        cpos;
  nacb::Quaternion  cquat;  
};



class InteractiveCameraView : public CameraCommon {
 public:

  InteractiveCameraView(){
    bdown = 0;
    mpos[0] = mpos[1] = 0;
  }

  virtual ~InteractiveCameraView(){ }

  virtual bool shiftActive() = 0;

  virtual bool mouse(int button, bool down, int x, int y){
    bdown = (down)?button+1:0;

    mpos[0] = x;
    mpos[1] = y;

    return true;
  }

  virtual void motion(int x, int y){
    double dx = x-mpos[0], dy=y-mpos[1];

    dx/=height();
    dy/=height();

    if(bdown == 2){
      if(shiftActive()){	
	double gain = 1.0;
	if(object)gain = tan(fov*M_PI/180/2.0)*(cpos.dot(cquat.rotate(nacb::Vec3d(0,0,1))))*2.0+0.01;
	cpos += cquat.rotate(nacb::Vec3d(-dx*gain, dy*gain,0));
      }
      else{ 
	if(object){
	  nacb::Vec3d focus(0,0,0);
	  nacb::Vec3d wo = cquat.conj().rotate(focus-cpos);
	  cquat = nacb::Quaternion::rod(nacb::Vec3d(0,-dx,0))* cquat * nacb::Quaternion::rod(nacb::Vec3d(-dy,0,0));
	  cpos = cquat.rotate(wo)*-1;
	}
	else {
	  cquat = nacb::Quaternion::rod(nacb::Vec3d(0,dx,0))* cquat * nacb::Quaternion::rod(nacb::Vec3d(dy,0,0));
	}
      } 
    }
    if(bdown==3){
      cpos += cquat.rotate(nacb::Vec3d(0,0,10.0*dy));
    }
    mpos[0] = x;
    mpos[1] = y;

    refresh();
  }
  virtual void refresh(){ }

 public:
  int bdown;
  int mpos[2];
};


class GLWindow : public InteractiveCameraView {
 public:
  // Incomplete, only one that matters is stencil
  enum {
    RGB_FLAG = 0x1,
    RGBA_FLAG = 0x2,
    ALPHA_FLAG = 0x4,
    DOUBLE_FLAG = 0x8,
    DEPTH_FLAG = 0x16,
    STENCIL_FLAG = 0x32,
    DEFAULT_FLAGS = RGBA_FLAG | ALPHA_FLAG | DEPTH_FLAG | DOUBLE_FLAG
  };

  GLWindow(int width, int height, int flags){
    initialize(width, height, flags);
  }
  
  GLWindow(int width=640, int height=480){
    initialize(width, height);
  }

  virtual ~GLWindow(){ 
    //Unregister the timer callback (if there was one).
    w_glutSetWindowData(0);
    glutTimerFunc(0, 0, 0);
  }
  
  void initialize(int width, int height, int flags = DEFAULT_FLAGS) {
    int ac = 1;
    char prog_name[] = {"program"};
    char * av = prog_name;

    printf("initting glut.\n");
    glutInit(&ac, &av);

    printf("setting window size.\n");
    glutInitWindowSize(width, height);

    int glutFlags = GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE;

    if (flags & STENCIL_FLAG)
      glutFlags |= STENCIL_FLAG;

    glutInitDisplayMode(glutFlags);
   
    printf("creating window.\n");
    glutCreateWindow("");

    printf("setting window data\n");
    w_glutSetWindowData(this);

    printf("setting other callbacks.\n");
    glutKeyboardFunc(staticKeyboard);
    glutKeyboardUpFunc(staticKeyboardUp);
    glutDisplayFunc(staticDisplay);

    printf("setting close func.\n");
    w_glutCloseFunc(staticClose);
    glutIdleFunc(staticIdle);
    glutEntryFunc(staticEntry);
    glutReshapeFunc(staticReshape);
    glutMotionFunc(staticMotion);
    glutPassiveMotionFunc(staticPassive);
    glutMouseFunc(staticMouse);

    printf("done calling display functions.\n");
  }

  virtual void draw(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    applyProjection();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    applyModelview();

    drawScene();

    swapBuffers();
  }
  virtual void drawScene(){
    glBegin(GL_QUADS);
    glVertex2d(-1.0, -1.0);
    glVertex2d( 1.0, -1.0);
    glVertex2d( 1.0,  1.0);
    glVertex2d(-1.0,  1.0);
    glEnd();
  }
  virtual void swapBuffers(){
    glutSwapBuffers();
  }
  virtual bool keyboardUp(unsigned char c, int x, int y){
    (void)x;
    (void)y;
    (void)c;
    return false;
  }
  virtual bool keyboard(unsigned char c, int x, int y){
    (void)x;
    (void)y;
    (void)c;
    return false;
  }
  virtual void idle(){ }
  virtual void closing(){
    closed_ = true;
  }
  virtual void entry(int v){ }
  virtual void reshape(int w, int h){
    glViewport(0, 0, w, h);
  }
  bool IsClosed() const { return closed_; }
  int width(){
    return glutGet(GLUT_WINDOW_WIDTH);
  }
  int height(){
    return glutGet(GLUT_WINDOW_HEIGHT);
  }
  bool shiftActive(){
    return mods & GLUT_ACTIVE_SHIFT;
  }
  void loop(bool start = true){
    if(start)
      glutMainLoop();
    else
      w_glutLeaveMainLoop();
  }

  //May need to have instance variable in future, for now it just calls
  //the timer func to refresh.
  void setRefreshRate(float fps){
    int frameTime = fps>0 ? floor(1.0/double(fps)): -100;
    glutTimerFunc(frameTime, staticTimerRefresh, frameTime);
  }

  void processEvent(){
    glutMainLoopEvent();
  }
  void refresh(){
    glutPostRedisplay();
  }
  static void staticKeyboardUp(unsigned char c, int x, int y){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data)
      data->keyboardUp(c, x, y);
  }
  static void staticKeyboard(unsigned char c, int x, int y){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data)
      data->keyboard(c, x, y);
  }
  static void staticDisplay(){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data)
      data->draw();
  }
  static void staticClose(){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data)
      data->closing();
  }
  static void staticIdle(){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data)
      data->idle();
  }
  static void staticEntry(int val){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data)
      data->entry(val);
  }
  static void staticReshape(int w, int h){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data) data->reshape(w, h);
  }
  static void staticMotion(int w, int h){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data) data->motion(w, h);
  }
  static void staticPassive(int w, int h){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data) data->motion(w, h);
  }
  static void staticMouse(int a, int b, int c, int d){
    GLWindow * data = (GLWindow *) w_glutGetWindowData();
    if(data){
      data->mods = glutGetModifiers();
      data->mouse(a, !b, c, d);
    }
  }
  static void staticTimerRefresh(int frameTime){
    if(frameTime>=0){
      GLWindow * data = (GLWindow *) w_glutGetWindowData();

      if(data)data->refresh();

      glutTimerFunc(frameTime, staticTimerRefresh, frameTime);
    }
  }
 protected:
  int                mods;
  int closed_ = false;
};

#endif //UTIGL_GLWIDNOW_H
