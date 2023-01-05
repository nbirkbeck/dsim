#ifndef FFONT_H
#define FFONT_H

#include "ft2build.h"
#include FT_FREETYPE_H
#include <stdio.h>
#include <GL/gl.h>
#include <string.h>
#include <memory>

class FFontData{
 protected:
  int h;
  float hintex;//the height in texture space
  float avgw;
  GLuint tex;
  float tc[128][4];
  float adv[128];
  float left[128];
  float top[128];
  float texiscale[2];
  float maxhratio;
 public:
  FFontData(const char * fname=0,unsigned int h=0){
    tex=0;
    glGenTextures(1,&tex);
    this->h=h;
    hintex=1;
    texiscale[0]=texiscale[1]=1;
    maxhratio=1;
    if(fname&&h){
      init(fname,h);
    }
  }
  void release(){
    if(tex)
      glDeleteTextures(1,&tex);
    h=0;
  }
  ~FFontData(){
    //twWarning(1,"Deleting Font Data!\n");
    release();
  }
  float getAvgWidth(){
    return avgw;
  }
  float getTexInvScaleX(){
    return texiscale[0];
  }
  float getTexInvScaleY(){
    return texiscale[1];
  }
  float getScaledMaxHeight(){
    return hintex*texiscale[1]*maxhratio;
  }
  float getScaledHeight(){
    return hintex*texiscale[1];
  }
  int getIndex(const char * str,
	       float xpos,float ypos,
	       float scx,float scy,float maxw);
  void getPosition(const char * str,int ind,
		   float & x,float & y,
		   float scx,float scy,float maxw);
  void getRequiredSize(const char * str,float & w,float & h,float scx,float scy,
		       float maxw=-1);
  float getWidth(char c,float scx);
  GLuint getTextureID(){
    return tex;
  }
  float drawString(const char * str,float & x,float & y,
		   float scx,float scy,float maxw,float maxh);
  void init(const char * fname,unsigned int h);

  void getTC(int index,float t[4]){
    memcpy(t,tc[index],sizeof(float)*4);
  }
  float getLeft(int index){
    return left[index];
  }
  float getTop(int index){
    return top[index];
  }
};
typedef std::shared_ptr<FFontData> FFontDataPtr;
class FFont{
 protected:
  float scale[2];
  float color[4];
 private:
  FFontDataPtr fontData;
 public:
  static const int lowest_char=32;
  static const int highest_char=127; 
  FFont(const char * name=0,unsigned int h=0){
    if(name){
      fontData.reset(new FFontData(name,h));
    }
    init();
  }
  void init(){
    scale[0]=1;
    scale[1]=1;
    color[0]=color[1]=color[2]=color[3]=1;
  }
  FFont(const FFont & font){
    this->fontData=font.fontData;
    memcpy(color,font.color,sizeof(float)*4);
    setScale(font.scale[0],font.scale[1]);
  }
  ~FFont(){
    
  }
  void setAlpha(float a){
    color[3]=a;
  }
  void setColor(float c[4]){
    color[0]=c[0];
    color[1]=c[1];
    color[2]=c[2];
    color[3]=c[3];
  }
  void setColor(float r,float g,float b,float a){
    color[0]=r;
    color[1]=g;
    color[2]=b;
    color[3]=a;
  }
  void scaleToFitWidth(int nchars,float width);
  void getColor(float * c){
    c[0]=color[0];
    c[1]=color[1];
    c[2]=color[2];
    c[3]=color[3];
  }
  float getR(){
    return color[0];
  }
  float getG(){
    return color[1];
  }
  float getB(){
    return color[2];
  }
  float getA(){
    return color[3];
  }
  void getScale(float * f){
    getScale(f[0],f[1]);
  }
  float getScaleY(){
    return scale[1];
  }
  float getScaleX(){
    return scale[0];
  }
  void getScale(float & x,float & y){
    x=scale[0];
    y=scale[1];
  }
  void setScale(float x,float y){
    scale[0]=x;
    scale[1]=y;
  }
  void multScale(float x,float y){
    scale[0]*=x;
    scale[1]*=y;
  }
  float getScaledMaxHeight(){
    return scale[1]*fontData->getScaledMaxHeight();
  }
  float getScaledHeight(){
    return scale[1]*fontData->getScaledHeight();
  }
  void scaleToBe(float pt,int win_height){
    //make sure the current scale is 1
    setScale(1,1);
    float newScale=2*(((float)pt)/(float)(win_height))/
      (fontData->getScaledHeight());
    setScale(newScale,newScale);
  }
  float drawString(const char * str,float * x,float * y,
		   float maxw=-1,float maxh=-1){
    glColor4fv(color);
    return fontData->drawString(str,*x,*y,scale[0],scale[1],maxw,maxh);
  }
  float drawString(const char * str,float x,float  y,
		   float maxw=-1,float maxh=-1){
    glColor4fv(color);
    return fontData->drawString(str,x,y,scale[0],scale[1],maxw,maxh);
  }
  float getWidth(const char * str){
    float x;
    float y;
    getRequiredSize(str,x,y);
    return x;
  }
  float getWidth(char c){
    return fontData->getWidth(c,scale[0]);
  }
  void getPosition(const char * str,int ind,
		   float & x,float & y,
		   float maxw=-1){
    fontData->getPosition(str,ind,x,y,scale[0],scale[1],maxw);
  }
  int getIndex(const char * str,
	       float xpos,float ypos,float maxw=-1){
    return fontData->getIndex(str,xpos,ypos,scale[0],scale[1],maxw);
  }
  void getRequiredSize(const char * str,float & x,float & y,
		       float maxw=-1){
    fontData->getRequiredSize(str,x,y,scale[0],scale[1],maxw);
  }
  void operator=(const FFont & other){
    fontData=other.fontData;
    memcpy(color,other.color,sizeof(float)*4);
    setScale(other.scale[0],other.scale[1]);
  }
  void getTC(int index,float t[4]){
    fontData->getTC(index,t);
  }
  float getTop(int index){
    return fontData->getTop(index);
  }
  float getLeft(int index){
    return fontData->getLeft(index);
  }
};

#endif
