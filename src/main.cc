#include <GL/glew.h>
#include <GL/gl.h>

#include "level.pb.h"
#include "math.h"
#include "proto.h"
#include "utigl/glwindow.h"
#include "utigl/ffont.h"
#include "level/intersection_control.h"
#include "level/parking_lot.h"
#include "level/road_segment.h"
#include "level/trip_stats.h"
#include "level/level.h"
#include "plan/plan.h"
#include "plan/stage.h"
#include "car.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <vector>
#include <deque>
#include <unordered_map>
#include <nmisc/timer.h>
#include <nmath/vec3.h>
#include <nmath/vec2.h>
#include <nimage/image.h>
#include <nappear/mesh.h>

DEFINE_string(filename, "", "Path to input filename");
DEFINE_string(model_dir, "", "Path to models");


class Renderer {
public:
  void Init() {
    ffont_ = FFont("/usr/share/fonts/bitstream-vera/Vera.ttf", 12);
    ffont_.setScale(0.5, 0.5);
  }

  virtual void DrawLevel(Level& level,
                         std::vector<Car>& cars) = 0;

protected:
  FFont ffont_;
};

class SimpleMeshRenderer: public Renderer {
public:
  void Init() {
    Renderer::Init();
    car_mesh_.readObj((FLAGS_model_dir + "/car.obj").c_str());
    car_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    car_mesh_.initNormals(true);

    wheel_mesh_.readObj((FLAGS_model_dir + "/wheel.obj").c_str());
    wheel_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    wheel_mesh_.initNormals(true);

    drop_shadow_mesh_.readObj((FLAGS_model_dir + "/drop_shadow.obj").c_str());
    drop_shadow_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    drop_shadow_mesh_.initNormals(true);

    stop_sign_mesh_.readObj((FLAGS_model_dir + "/stop_sign.obj").c_str());
    stop_sign_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    stop_sign_mesh_.initNormals(true);

    lights_mesh_.readObj((FLAGS_model_dir + "/lights.obj").c_str());
    lights_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    lights_mesh_.initNormals(true);

    light_mesh_.readObj((FLAGS_model_dir + "/light.obj").c_str());
    light_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    light_mesh_.initNormals(true);

    tail_light_mesh_.readObj((FLAGS_model_dir + "/tail_light.obj").c_str());
    tail_light_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    tail_light_mesh_.initNormals(true);
    
    glGenTextures(1, &tex_);
    nacb::Image8 image;
    image.read((FLAGS_model_dir + "/car.png").c_str());
    glBindTexture(GL_TEXTURE_2D, tex_);
    image.initTexture();

  }
  void DrawGroundPlane() {
    glColor3f(0.05, 0.1, 0.05);
    glDepthMask(0);
    glBegin(GL_QUADS);
    glVertex2f(-100, -100);
    glVertex2f( 100, -100);
    glVertex2f( 100,  100);
    glVertex2f(-100,  100);
    glEnd();
    glDepthMask(1);
  }

  void DrawIntersection(const IntersectionControl& intersection) {
    glPushMatrix();
    glTranslatef(intersection.pos.x,
                 intersection.pos.y,
                 0);
    if (intersection.IsStopSign()) {
      glColor3f(1, 0.25, 0.25);
      const double r = atan2(intersection.dir.y,
                             intersection.dir.x);
      glRotatef(180 * r / M_PI, 0, 0, 1);
      glTranslatef(-0.5, -0.5, 0);
      stop_sign_mesh_.draw();
    } else if (intersection.IsLight()) {
      glColor3f(0.5, 0.5, 0.5);
      lights_mesh_.draw();

      for (const auto& segment : intersection.incoming_segments) {
        const auto& points = segment.first->points;
        const nacb::Vec2d& p1 = points[segment.second];
        const nacb::Vec2d& p2 = points[(segment.second + 1) % points.size()];
        const nacb::Vec2d dir = p2 - p1;
        const double r = atan2(dir.y, dir.x);
        glPushMatrix();
        glRotatef(180 * r / M_PI, 0, 0, 1);
        if (intersection.IsGreen(segment.first)) {
          glColor3f(0.1, 1, 0.1);
        } else {
          glColor3f(1, 0.1, 0.1);
        }
        light_mesh_.draw();
        glPopMatrix();
      }
    }
    glPopMatrix();
  }

  void DrawParkingLot(const ParkingLot& lot) {
    glColor3f(1, 1, 1);
    glPushMatrix();
    glTranslatef(lot.pos.x,  lot.pos.y, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(-lot.width() / 2, -lot.height() / 2);
    glVertex2f(-lot.width() / 2,  lot.height() / 2);
    glVertex2f( lot.width() / 2,  lot.height() / 2);
    glVertex2f( lot.width() / 2, -lot.height() / 2);
    glEnd();

    glPointSize(1);
    glBegin(GL_POINTS);
    for (int k = 0; k < (int)lot.parking_spots.size(); ++k) {
      const auto& p = lot.parking_spots[k].pos;
      glVertex2f(p.x, p.y);
    }
    glEnd();
    glPopMatrix();
  }

  void DrawRoadSegment(RoadSegment& segment) {
    glColor3f(1, 0, 0);
    glBegin(GL_LINE_STRIP);
    for (const auto& co : segment.points) {
      glVertex2f(co.x, co.y);
    }
    glEnd();

    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    const double speed = segment.GetAverageSpeed();
    const nacb::Vec2d co = (segment.points[0] + segment.points[1]) * 0.5;
    char str[1024];
    snprintf(str, sizeof(str), "%3.2f", speed);
    ffont_.drawString(str, co.x, co.y - 0.1);
    glDisable(GL_TEXTURE_2D);
  }

  void DrawCar(const Car& car) {
    glColor3f(0.5, 0.5, 0.5);
    glPushMatrix();
    double r = car.angle_;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex_);

    glColor3f(0.5 + (car.car_id_ % 32) / 64.0,
              0.75 - (car.car_id_ % 32) / 64.0,
              0.25 + (car.car_id_ % 33) / 64.0);
    glTranslatef(car.pos().x, car.pos().y, 0);
    glRotatef(180 * r / M_PI, 0, 0, 1);
    car_mesh_.draw();

    glDisable(GL_TEXTURE_2D);

    if (car.breaking_ >= 1) {
      glDisable(GL_LIGHTING);
      glColor3f(1.0, 0.0, 0.0);
      tail_light_mesh_.draw();
      glEnable(GL_LIGHTING);
    }
      
    glColor3f(0.2, 0.2, 0.2);
    glPushMatrix();
    glTranslatef(0.85/4, -1.0/4.0, 0.4/4);
    r = 0.39 / 4.0 * 2 * M_PI;
    glRotatef(car.wheel_rot_ * 180.0 / M_PI, 0, 0, 1);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.85/4,  1.0/4.0, 0.4/4);
    glRotatef(car.wheel_rot_ * 180.0 / M_PI, 0, 0, 1);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-0.85/4,  -1.0/4.0, 0.4/4);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-0.85/4,  1.0/4.0, 0.4/4);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw();
    glPopMatrix();

    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glColor3f(1, 1, 1);
    drop_shadow_mesh_.draw();
    glEnable(GL_LIGHTING);

    glPopMatrix();
  }

  void DrawLevel(Level& level,
                 std::vector<Car>& cars) {
    DrawGroundPlane();
    
    for (int i = 0; i < (int)level.intersections.size(); ++i) {
      DrawIntersection(level.intersections[i]);
    }
    
    for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
      DrawParkingLot(level.parking_lots[i]);
    }

    for (int i = 0; i < (int)level.road_segments.size(); ++i) {
      DrawRoadSegment(level.road_segments[i]);
    }

    if (car_mesh_.vert.empty()) {
      glPointSize(3);
      glBegin(GL_POINTS);
      for (const auto& car: cars) {
        glColor3f(0, 1, 0);
        if (car.breaking_ > 1) {
          glColor3f(0, 0, 1);
        }
        glVertex2f(car.pos().x, car.pos().y);
      }
      glEnd();
    }
    
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHT0);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    
    float pos[4] = {0, 0, 1, 0};
    float white[4] = {1, 1, 1, 1};
    float black[4] = {0, 0, 0, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_AMBIENT, black);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, white);
    const GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      LOG(INFO) << err;
    }
    for (const auto& car: cars) {
      DrawCar(car);
    }
    glDisable(GL_LIGHTING);

    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    double average_trip_time = level.stats.GetAverageTripTime();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0, 0, -20);
    char str[1024];
    snprintf(str, sizeof(str), "ATT: %f", average_trip_time);
    ffont_.drawString(str, -8.f, -6.f);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
  }

protected:
  nappear::Mesh car_mesh_;
  nappear::Mesh wheel_mesh_;
  nappear::Mesh drop_shadow_mesh_;
  nappear::Mesh stop_sign_mesh_;
  nappear::Mesh lights_mesh_;
  nappear::Mesh light_mesh_;
  nappear::Mesh tail_light_mesh_;
  GLuint tex_ = 0;
};


class LevelWindow: public GLWindow {
public:
  LevelWindow(Level& level) : level_(level)  {
    cpos.z = 100;
    farPlane = 1000;
    setRefreshRate(60);

    if (false) {
      for (int i = 0; i < 7; ++i) {
        cars_.push_back(Car(level, i, (i) % 7, ((i) + 1) % 7));
      }
    } else {
      for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
        for (int j = 0; j < (int)level.parking_lots[i].parking_spots.size(); ++j) {
          cars_.push_back(Car(level, cars_.size(), i, (i + 1 + rand()) % level.parking_lots.size()));
        }
      }
    }
    glClearColor(0.5, 0.5, 0.7, 1.0);
    level_renderer_.Init();
  }

  bool keyboard(unsigned char c, int x, int y) {
    if (c == 'f') {
      follow_ = !follow_;
    }
    return true;
  }

  void drawScene() {
    static int count = 0;
    if (count % 100 == 0) {
      level_.stats.Print();
    }
    ++count;

    if (follow_) {
      auto& car = cars_[0];
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      double r = car.angle_;

      glRotatef(15, 1, 0, 0);
      glTranslatef(0, -1.0, -1.0);
      glRotatef(90, -1, 0, 0);
      glRotatef(-180 * r / M_PI + 90, 0, 0, 1);
      glTranslatef(-car.pos().x, -car.pos().y, 0);
    }

    level_renderer_.DrawLevel(level_, cars_);
  }

  void refresh() {
    static nacb::Timer timer;
    const double dt = (double)timer;
    timer.reset();
    if (dt > 0) {
      for (auto& car: cars_) {
        car.Step(cars_, t_, dt);
      }
      for (auto& isect: level_.intersections) {
        isect.Step(dt);
      }
      t_ += dt;
    }
    GLWindow::refresh();
  }
  double t_ = 0;
  Level& level_;
  std::vector<Car> cars_;
  bool follow_ = false;
  SimpleMeshRenderer level_renderer_;

};

int main(int ac, char* av[]) {
  gflags::ParseCommandLineFlags(&ac, &av, true);
  google::InitGoogleLogging(av[0]);

  LOG(INFO) << "Loading filename:" << FLAGS_filename;
  dsim::Level level_data;
  if (!Proto::ReadProto<dsim::Level>(FLAGS_filename, &level_data)) {
    LOG(ERROR) << "Unable to load level from:" << FLAGS_filename;
  }

  LOG(INFO) << level_data.DebugString();

  Level level(level_data);
  for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
    for (int j = 0; j < (int)level.parking_lots.size(); ++j) {
      if (i == j) continue;
      auto plan = plan::PlanTravel(level, level.parking_lots[i], 0,
                                   level.parking_lots[j], 0);
      std::cout << plan.size();
    }
  }

  LevelWindow level_window(level);
  glewInit();
  
  level_window.loop(1);
  return 0;
}
