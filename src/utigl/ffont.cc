#include "ffont.h"
#include <freetype/ftglyph.h>
#include <algorithm>
#include <math.h>
#include <nimage/image.h>

using NIMAGE_NSPACE::Image8;

//Testing out the magnification of textures distance function
//for alpha.  Should set a mipmap level to this.
//#define ALPHA_MAG

#ifdef ALPHA_MAG
#define CHAR_PADDING 2*8
#else
#define CHAR_PADDING 2  //old value
#endif


//next largest power of 2
// http://aggregate.org/MAGIC/#Next%20Largest%20Power%20of%202
unsigned int nlpo2(unsigned int x)
{
  x|=(x>>1);x|=(x>>2);
  x|=(x>>4);x|=(x>>8);
  x|=(x>>16);return(x+1);
}

int enoughRoom(int dims[][2],int n_char,int maxh,
	       int pw,int ph)
{
  int curx=1;
  int cury=1;
  for(int c=0;c<n_char;c++){
    if(curx+dims[c][0]+1>=pw){
      curx=1;
      cury+=maxh+CHAR_PADDING;
    }
    //try to place
    if(cury+maxh>=ph)return 0;
    curx+=CHAR_PADDING+dims[c][0];
  }
  return 1;
}
void FFontData::init(const char * fname, unsigned int h) {
  //twLog(combine("%s %d",fname,h));

  if(tex){
    glGenTextures(1,&tex);
  }  
  this->h=h;
 
  FT_Library library;
  if (FT_Init_FreeType( &library )){
    fprintf(stderr,"init free type failed\n");
  }
  FT_Face face;
  if (FT_New_Face( library, fname, 0, &face )){
    fprintf(stderr,"%s:%d, error loading file: %s\n", __FILE__, __LINE__, fname);
    return;
  }

#ifdef ALPHA_MAG
  const int ups = 8;
  h = h * ups; //Test out the alpha-magnified texture
#endif  

  FT_Set_Char_Size( face, h << 6, h << 6, 96, 96);
  
  //char lowest_char=32;
  //char highest_char=127;
  char n_char=FFont::highest_char-FFont::lowest_char;

  Image8 im[n_char];
  int dims[n_char][2];
  int maxw=0;
  int maxh=0;
  int totalw=0;
  
  for(unsigned char c=FFont::lowest_char;c<FFont::highest_char;c++){
    if(FT_Load_Glyph( face, FT_Get_Char_Index( face, c ), FT_LOAD_DEFAULT ))
      fprintf(stderr,"FT_Load_Glyph failed");
    
    FT_Glyph glyph;
    if(FT_Get_Glyph( face->glyph, &glyph ))
      fprintf(stderr,"Get glyph failed, char %c\n",c);
    
    FT_Glyph_To_Bitmap( &glyph, ft_render_mode_normal, 0, 1 );
    FT_BitmapGlyph bitmap_glyph = (FT_BitmapGlyph)glyph;
    
    int clow=c-FFont::lowest_char;

    FT_Bitmap& bitmap=bitmap_glyph->bitmap;
    //make sure that it is at least 1by h
    float advw=bitmap_glyph->root.advance.x/(float)(1<<16);
    im[clow]=Image8(std::max((int)bitmap.width,1),std::max((int)h,(int)bitmap.rows),4);
    im[clow]=0;
    dims[clow][0]=bitmap.width;//(int)(advw);//bitmap.width;
    dims[clow][1]=bitmap.rows;
    left[clow]=bitmap_glyph->left;
    top[clow]=(int)bitmap_glyph->top - (int)bitmap.rows;
    adv[clow]=advw;
    //printf("bitmap rows %d, h:%d\n",bitmap.rows,h);
    maxw=std::max(maxw,(int)bitmap.width);
    maxh=std::max(maxh,(int)bitmap.rows);
    totalw+=bitmap.width;

    for(int i=0;i<(int)bitmap.rows;i++){
      for(int j=0;j<(int)bitmap.width;j++){
	im[clow](j,i,0)=im[clow](j,i,1)=im[clow](j,i,2)=255;
	//color used to be:(bitmap.buffer[i*bitmap.pitch+j]>0)*255;
	im[clow](j,i,3)=bitmap.buffer[i*bitmap.pitch+j];
      }
    }
    FT_Done_Glyph( glyph );
  }

  int required_space=(totalw+CHAR_PADDING*n_char)*(maxh+CHAR_PADDING);
  printf("required_space = %d sqrt(%f)\n",required_space,ceil(sqrt(required_space)));
  int ph=nlpo2((int)ceil(sqrt(required_space)))>>1;
  int pw=ph;
  for(int i=0;i<32;i++){
    if(enoughRoom(dims,n_char,maxh,pw,ph))
      break;
    pw*=2;
    if(enoughRoom(dims,n_char,maxh,pw,ph))
      break;    
    pw/=2;
    ph*=2;
    if(enoughRoom(dims,n_char,maxh,pw,ph))
      break;    
    pw*=2;
  }
  printf("%d by %d\n",pw,ph);
  Image8 packedim(pw,ph,4);

  int curx=1;
  int cury=1;
  texiscale[0]=((float)pw)/((float)h);
  texiscale[1]=((float)ph)/((float)h);
 
  hintex=(float)h/((float)ph);
  packedim=0;
  //Default is now a white image with zero alpha
  for(int y=0;y<packedim.h;y++){
    for(int x=0;x<packedim.w;x++){
      packedim(x,y,0)=packedim(x,y,1)=packedim(x,y,2)=255;
      packedim(x,y,3)=0;
    }
  }

  avgw=0;//compute the average width
  printf("maxh is %d, h is %d\n",maxh,h);

  this->maxhratio=((float)maxh)/((float)h);

  for(int c=0;c<n_char;c++){
    if(curx+dims[c][0]+1>=pw){
      curx=1;
      cury+=maxh+CHAR_PADDING;//Using maxh here ensures no overlap in texture space
    }
    packedim.setSubimage(curx,cury,im[c]);
    
    //setup the texture coordinates
    tc[c][0]=((float)curx)/(float)(pw); //used to be curx/pw
    tc[c][1]=((float)cury)/(float)(ph); //used to be cury/ph
    
    tc[c][2]=(dims[c][0])/(float)(pw); //used to div pw-1
    tc[c][3]=(dims[c][1])/(float)(ph); //used to div ph-1
    
    left[c]/=(float)(pw); //used to div pw-1
    top[c]/=(float)(ph);  //used to div ph-1

    adv[c]/=(float)(pw);

    avgw+=tc[c][2];
    curx+=dims[c][0]+CHAR_PADDING;
  }
  avgw/=n_char;
  glBindTexture(GL_TEXTURE_2D,tex);
#ifdef FFONT_TEST
  packedim.writeTGA("packedim.tga");
#endif

#ifdef ALPHA_MAG
  nacb::Imagef distOut = packedim.edt_8sed(3);
  distOut = (distOut+2.0*float(ups))/(4.0f*ups);
  for(int i=0; i<distOut.w*distOut.h; i++)
    distOut[i] = std::max(0.0f, std::min(distOut[i], 1.0f));

  float mul = ups;
  float offs = mul/2.0f - 0.5f;
  nacb::Image8 pimnew(packedim.w/ups, packedim.h/ups, 4);
  pimnew = 255;
  for(int y=0; y<pimnew.h; y++){
    for(int x=0; x<pimnew.w; x++){
      float t = distOut.bilinear(mul*x+offs, mul*y+offs, 0);
      pimnew(x, y, 3) = (char)std::max(0.0f, std::min(t*255.0f, 255.0f));
    }
  }
  packedim = pimnew;// packedim.resize(packedim.w/8, packedim.h/8);
  packedim.write("/tmp/pim.tga");
#endif
  //printf("texture size is %dx%d\n",packedim.w,packedim.h);
  packedim.initTexture();//GL_MODULATE,0);
  //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  
  //free up the free type stuff
  FT_Done_Face(face);
  FT_Done_FreeType(library);
  

  //twLog(combine("returning, font texture is %d",tex));
}
float FFontData::getWidth(char c,float scx){
  c-=FFont::lowest_char;
  float charw=adv[(int)c];///tc[c][2]; //Changed to adv Aug 31/07, makes more sens so that users can use getWidth of a char to determine length of a string
  return charw*texiscale[0]*scx;
}
//added bounds checking Jan 24/07
int FFontData::getIndex(const char * str,
			float xpos,float ypos,
			float scx,float scy,float maxw){
  float x=0;
  float y=0;
  int len=strlen(str);
  float fonth=hintex*texiscale[1]*scy*maxhratio;
  for(int i=0;i<len;i++){
    char c=str[i]-FFont::lowest_char;
    int inbounds=(str[i]>=FFont::lowest_char&&str[i]<FFont::highest_char);
    float charw=(inbounds)?adv[(int)c]:0;
    float xbefore;
    
    if(*str==10||*str==13){
      x=0;//Added Jan 24/07
      y-=hintex*texiscale[1]*scy*maxhratio;
    }
    if(inbounds && maxw>0 && x+(charw+left[(int)c])*scx*texiscale[0]>maxw){
      x=0;
      y-=hintex*texiscale[1]*scy*maxhratio;
    }
    xbefore=x;
    x+=charw*scx*texiscale[0];

    if(xpos>=xbefore && xpos<x &&
       ypos>=y && ypos<y+fonth)
      return i;
  }
  return len;
}
void FFontData::getPosition(const char * str,int ind,
			    float & x,float & y,
			    float scx,float scy,float maxw){
  int len=strlen(str);
  ind=std::min(ind,len);
  x=0;
  y=0;
  for(int i=0;i<ind;i++){
    char c=str[i]-FFont::lowest_char;
    int inbounds=(str[i]>=FFont::lowest_char&&str[i]<FFont::highest_char);
    float charw=(inbounds)?adv[(int)c]:0;

    if(*str==10||*str==13){
      x=0;//Added Jan 24/07
      y-=hintex*texiscale[1]*scy*maxhratio;
    }
    if(inbounds && maxw>0 && x+(charw+left[(int)c])*scx*texiscale[0]>maxw){
      x=0;
      y-=hintex*texiscale[1]*scy*maxhratio;
    }
    x+=charw*scx*texiscale[0];;
  }
}
void FFontData::getRequiredSize(const char * str,float & w,float & h,float scx,float scy,float maxw)
{
  float thisw=0;
  w=0;
  h=hintex*texiscale[1]*scy*maxhratio;
  
  while(*str){
    char c=str[0]-FFont::lowest_char;

    int inbounds=(str[0]>=FFont::lowest_char&&str[0]<FFont::highest_char);
    //Make sure it is in range before trying to index stuff with 'c'
    float charw=(inbounds)?adv[(int)c]:0;
    if(*str==10||*str==13){
      if(thisw>w)w=thisw;
      thisw=0;
      h+=hintex*texiscale[1]*scy*maxhratio;
    }
    if(inbounds && maxw>0 && thisw+(charw+left[(int)c])*scx*texiscale[0]>maxw){
      if(thisw>w)w=thisw;
      thisw=0;
      h+=hintex*texiscale[1]*scy*maxhratio;
    }
    thisw+=charw*scx*texiscale[0];
    str++;
  }
  if(thisw>w)w=thisw;

  if(maxw>0){
    w=maxw;
    h=-(h-hintex*texiscale[1]*scy*maxhratio);//remove the upper, and negate
  }
}
float FFontData::drawString(const char * str,float & x,float & y,
			    float scalex,float scaley,
			    float maxw,float maxh)
{
  float xback=x;
  float yback=y;

  float alphaRef;
  glGetFloatv(GL_ALPHA_TEST_REF, &alphaRef);
  glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D,tex);
  glEnable(GL_ALPHA_TEST);
#ifdef ALPHA_MAG
  glDisable(GL_BLEND);  
  glAlphaFunc(GL_GREATER,0.5);
#else
  glEnable(GL_BLEND);  
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
#endif
    
  glPushMatrix();
  glTranslatef(x,y,0);
  glScalef(scalex*texiscale[0],
	   scaley*texiscale[1],1);
  
  if(maxw>0)maxw/=(scalex*texiscale[0]);
  if(maxh>0)maxh/=(scaley*texiscale[1]);

  x=0;
  y=0;
  int onthisline=0;
  glBegin(GL_QUADS);
  while(*str){
    if(*str==10||*str==13){
      x=0;
      y-=maxhratio*hintex;
      str++;
      onthisline=0;
      continue;
    }
    if(str[0]<FFont::lowest_char || str[0]>=FFont::highest_char){
      str++;
      continue;
    }
    int c=str[0]-FFont::lowest_char;
    float w=tc[c][2];
    float h=tc[c][3];
    float tcx=tc[c][0];
    float tcy=tc[c][1];
    float usex=x+left[c];
    float usey=y+top[c];

    //Using adv[c] here to be consistent with the getPosition
    if(maxw>0 && usex+adv[c]>maxw){
      //if we didn't write anything, then we have run out of space
      //completely!
      if(onthisline==0)break;
      y-=maxhratio*hintex;
      usey-=maxhratio*hintex;
      x=0;
      usex=0;
    }
    if(maxh>0 && usey<-maxh){
      break;
    }

    glTexCoord2f(tcx,tcy+h);
    glVertex2f(usex,usey);
    glTexCoord2f(tcx+w,tcy+h);
    glVertex2f(usex+w,usey);
    glTexCoord2f(tcx+w,tcy);
    glVertex2f(usex+w,usey+h);
    glTexCoord2f(tcx,tcy);
    glVertex2f(usex,usey+h);

    str++;
    x+=adv[c];
    onthisline++;
  }
  glEnd();
  
  glPopMatrix();
  glPopAttrib();
  glAlphaFunc(GL_GREATER, alphaRef);
  x=xback+x*scalex*texiscale[0];//add back the x and y components
  y=yback+y*scaley*texiscale[1];
  return hintex*scaley*texiscale[1];
}

void FFont::scaleToFitWidth(int nchars,float width)
{
  if(!fontData){
    printf("FFont::scaleToFitWidth(%s,%d) no fontData\n",
	   __FILE__,__LINE__);
    return;
  }
  float sc=width/(nchars*fontData->getAvgWidth()*fontData->getTexInvScaleX());
  setScale(sc,sc);
}

#ifdef FFONT_TEST
#include <GL/glut.h>
#include "fbo.h"

int main(int ac,char * av[])
{
  if(ac>1){
    /*
      This should settle it for once and for all: the texture
      coordinates are good when using GL_LINEAR texture filter!
      
      Read in the texture map (packeim.tga) and compare this to
      the rendered version.

      Since blending is enabled the r,g,b color channels of the result
      are going to be less than 255, but they should be the same as 
      the alpha channel from the texture image...and they pretty
      much are (less than a pixel).
    */
    glutInit(&ac,av);
    glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH|GLUT_ALPHA);
    glutInitWindowSize(32,32);
    glutCreateWindow("unused");
    FFont font(av[1],atoi(av[2]));

    Image8 im;
    im.readTGA("packedim.tga");

    FrameBufferObject fbo(im.w,im.h);
    fbo.bind(1);
    glViewport(0,0,im.w,im.h);
    glClearColor(0,0,0,0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0,im.w,im.h,0,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glColor3f(1,1,1);
    font.setScale(atoi(av[2]),atoi(av[2]));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    double mindiff=100000;
    glClear(GL_COLOR_BUFFER_BIT);
    
    for(int i=FFont::lowest_char;i<FFont::highest_char;i++){
      float tc[4];
      char str[2]=" ";
      str[0]=i;
      font.getTC(i-FFont::lowest_char,tc);
      float l=font.getLeft(i-FFont::lowest_char);
      float t=font.getTop(i-FFont::lowest_char);

     
      printf("%c tc=[%f %f %f %f], l:%f t:%f   %f %f   %f l:%f t:%f\n", i,
             tc[0], tc[1], tc[2], tc[3], l, t,
             (tc[0]-l)*im.w, (1-(tc[1]+(tc[3])+t))*im.h,
             tc[2]*im.w, l*im.w, t*im.h);
      font.drawString(str,(tc[0]-l)*im.w,(1-(tc[1]+tc[3]+t))*im.h);
    }
    
    Image8 read(im.w,im.h,4);
    Image8 dif(im.w,im.h,1);
    glReadPixels(0,0,im.w,im.h,read.channelToGL(),read.typeToGL(),read.data);
    
    //Compare the alpha channel to the image channel
    double avgd=0;
    for(int y=0;y<im.h;y++){
      for(int x=0;x<im.w;x++){
	double r=double(im(x,y,3))-double(read(x,y,0));
	avgd+=r*r;

	dif(x,y)=(r*10);
	
	//read(x,y,0)=read(x,y,1)=read(x,y,2)=255;
      }
    }
    avgd/=(im.w*im.h);
    avgd=sqrt(avgd);

    printf("average diff %f\n",avgd);
    read.writeTGA("read.tga");

    im.getChannel(3).write("im.png");
    read.getChannel(0).write("read.png");
    dif.write("diff.png");
    /****
    GLuint tex;
    glClear(GL_COLOR_BUFFER_BIT);
    glGenTextures(1,&tex);

    printf("tex is %d\n",tex);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,tex);
    im.initTexture();
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glDisable(GL_ALPHA_TEST);
    //glEnable(GL_ALPHA_FUNC);

    glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
    

    glDisable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glColor3f(1,1,1);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0,im.w,0,im.h,-1,1);

    int win=8;
    double mind=1e10;
    for(double ups=-1;ups<=1;ups+=1.0/16.0){
      for(double offs=-1;offs<=1;offs+=1.0/16.0){
	glClear(GL_COLOR_BUFFER_BIT);
	for(int y=0;y<im.h;y+=win){
	  for(int x=0;x<im.w;x+=win){
	    float tcx0=(float(x)+offs)/float(im.w);
	    float tcx1=tcx0+(float(win)+ups)/im.w;
	    float tcy0=(float(y)+offs)/float(im.h);
	    float tcy1=tcy0+(float(win)+ups)/im.h;
	    
	    float xf=(float)x;
	    float yf=(float)y;
	    
	    glBegin(GL_QUADS);
	    glTexCoord2f(tcx0,tcy0);
	    glVertex2f(xf,yf);
	    glTexCoord2f(tcx1,tcy0);
	    glVertex2f(xf+win,yf);
	    glTexCoord2f(tcx1,tcy1);
	    glVertex2f(xf+win,yf+win);
	    glTexCoord2f(tcx0,tcy1);
	    glVertex2f(xf,yf+win);
	    glEnd();
	  }
	}
	glReadPixels(0,0,im.w,im.h,read.channelToGL(),read.typeToGL(),read.data);
	//read.writeTGA("read.tga");
	
	avgd=0;
	for(int y=0;y<im.h;y++){
	  for(int x=0;x<im.w;x++){
	    double r=double(im(x,y,0))-double(read(x,y,0));
	    avgd+=r*r;
	  }
	}
	avgd/=(im.w*im.h);
	avgd=sqrt(avgd);
	if(avgd<mind){
	  mind=avgd;
	  printf("average is %f (offs=%f, ups=%f)\n",avgd,offs,ups);
	  read.writeTGA("read.tga");
	}
      }
    }
    ***/
    fbo.bind(0);

    //read.writeTGA("read.tga");
  }
  else
    printf("run with two argument ./a.out fontfile height\n");
  return 0;
}
#endif
