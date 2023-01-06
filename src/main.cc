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
DEFINE_string(benchmark, "", "Benchmark to run (plan, render, sim)");


class Renderer {
public:
  void Init() {
    ffont_ = FFont("/usr/share/fonts/bitstream-vera/Vera.ttf", 12);
    ffont_.setScale(0.25, 0.25);
  }

  virtual void DrawLevel(const nacb::Quaternion& cquat, Level& level, std::vector<Car>& cars) = 0;

protected:
  FFont ffont_;
};


class VBOMesh {
public:
  VBOMesh() {

  }
  void Init(const nappear::Mesh& mesh) {
    std::vector<float> vertex_data;
    std::vector<float> uv_data;
    std::vector<float> normal_data;
    for (int i = 0; i < (int)mesh.faces.size(); ++i) {
      for (int k = 0; k < 3; ++k) {
        vertex_data.push_back(mesh.vert[mesh.faces[i].vi[k]].x);
        vertex_data.push_back(mesh.vert[mesh.faces[i].vi[k]].y);
        vertex_data.push_back(mesh.vert[mesh.faces[i].vi[k]].z);

        normal_data.push_back(mesh.norm[mesh.faces[i].ni[k]].x);
        normal_data.push_back(mesh.norm[mesh.faces[i].ni[k]].y);
        normal_data.push_back(mesh.norm[mesh.faces[i].ni[k]].z);

        uv_data.push_back(mesh.tvert[mesh.faces[i].tci[k]].x);
        uv_data.push_back(mesh.tvert[mesh.faces[i].tci[k]].y);
      }
    }

    std::vector<float> vbo_data(vertex_data.size() + uv_data.size() + normal_data.size());
    std::copy(vertex_data.begin(), vertex_data.end(), vbo_data.begin());
    std::copy(normal_data.begin(), normal_data.end(), &vbo_data[vertex_data.size()]);
    std::copy(uv_data.begin(), uv_data.end(), &vbo_data[vertex_data.size()  + normal_data.size()]);
    
    glGenBuffers(1,&vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vbo_data.size(), &vbo_data[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    normal_offset = sizeof(float) * (vertex_data.size());
    uv_offset = sizeof(float) * (vertex_data.size() + normal_data.size());
    num_faces = mesh.faces.size();
  }

  bool empty() {
    return num_faces == 0;
  }

  void Bind(bool bind) {
    if (bind) {
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_NORMAL_ARRAY);
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);

      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glVertexPointer(3, GL_FLOAT, 0, 0);
      glNormalPointer(GL_FLOAT, 0,((unsigned char *)(0))+normal_offset);
      glTexCoordPointer(2, GL_FLOAT, 0,((unsigned char *)(0))+uv_offset);
    } else {
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glDisableClientState(GL_VERTEX_ARRAY);
      glDisableClientState(GL_NORMAL_ARRAY);
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }
  }

  void draw(bool bind=true) {
    if (bind) Bind(true);
    glDrawArrays(GL_TRIANGLES, 0, num_faces * 3);
    if (bind) Bind(false);
  }
private:
  GLuint vbo = 0;
  int normal_offset = 0;
  int uv_offset = 0;
  int num_faces = 0;
};

class SimpleMeshRenderer: public Renderer {
public:
  void Init() {
    Renderer::Init();

    nappear::Mesh car_mesh;
    car_mesh.readObj((FLAGS_model_dir + "/car.obj").c_str());
    car_mesh.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    car_mesh.initNormals(true);
    car_mesh_.Init(car_mesh);

    nappear::Mesh wheel_mesh;
    wheel_mesh.readObj((FLAGS_model_dir + "/wheel.obj").c_str());
    wheel_mesh.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    wheel_mesh.initNormals(true);
    wheel_mesh_.Init(wheel_mesh);

    nappear::Mesh drop_shadow_mesh;
    drop_shadow_mesh.readObj((FLAGS_model_dir + "/drop_shadow.obj").c_str());
    drop_shadow_mesh.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    drop_shadow_mesh.initNormals(true);
    drop_shadow_mesh_.Init(drop_shadow_mesh);
    
    stop_sign_mesh_.readObj((FLAGS_model_dir + "/stop_sign.obj").c_str());
    stop_sign_mesh_.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    stop_sign_mesh_.initNormals(true);

    lights_mesh_.readObj((FLAGS_model_dir + "/lights.obj").c_str());
    lights_mesh_.scaleTranslate(nacb::Vec3f(0.5, 0.5, 0.5), nacb::Vec3f(0, 0, 0));
    lights_mesh_.initNormals(true);

    light_mesh_.readObj((FLAGS_model_dir + "/light.obj").c_str());
    light_mesh_.scaleTranslate(nacb::Vec3f(0.5, 0.5, 0.5), nacb::Vec3f(0, 0, 0));
    light_mesh_.initNormals(true);

    nappear::Mesh tail_light_mesh;
    tail_light_mesh.readObj((FLAGS_model_dir + "/tail_light.obj").c_str());
    tail_light_mesh.scaleTranslate(nacb::Vec3f(0.25, 0.25, 0.25), nacb::Vec3f(0, 0, 0));
    tail_light_mesh.initNormals(true);
    tail_light_mesh_.Init(tail_light_mesh);
    
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
    glVertex2f(-200, -200);
    glVertex2f( 200, -200);
    glVertex2f( 200,  200);
    glVertex2f(-200,  200);
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

    glLineWidth(2);
    glPushMatrix();
    glTranslatef(lot.pos.x,  lot.pos.y, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(-lot.width() / 2, -lot.height() / 2);
    glVertex2f(-lot.width() / 2,  lot.height() / 2);
    glVertex2f( lot.width() / 2,  lot.height() / 2);
    glVertex2f( lot.width() / 2, -lot.height() / 2);
    glEnd();
    glLineWidth(1);

    glDepthMask(0);

    glColor3f(0.2, 0.2, 0.2);
    glBegin(GL_QUADS);
    glVertex2f(-lot.width() / 2, -lot.height() / 2);
    glVertex2f(-lot.width() / 2,  lot.height() / 2);
    glVertex2f( lot.width() / 2,  lot.height() / 2);
    glVertex2f( lot.width() / 2, -lot.height() / 2);
    glEnd();

    glPointSize(1);
    glColor3f(1, 1, 1);
    glBegin(GL_POINTS);
    for (int k = 0; k < (int)lot.parking_spots.size(); ++k) {
      const auto& p = lot.parking_spots[k].pos;
      glVertex2f(p.x, p.y);
    }
    glEnd();
    glDepthMask(1);


    glPopMatrix();
  }

  std::vector<nacb::Vec2d> GetRoadSegmentNormals(RoadSegment& segment) {
    std::vector<nacb::Vec2d> dirs(segment.points.size(), nacb::Vec2d(0, 0));

    for (int i = 0; i < int(segment.points.size()) - 1; ++i) {
      const nacb::Vec2d& p1 = segment.points[i];
      const nacb::Vec2d& p2 = segment.points[i + 1];
      nacb::Vec2d d = (p2 - p1);
      d.normalize();
      d = nacb::Vec2d(-d.y, d.x);
      dirs[i] += d;
      if (i > 0) {
        dirs[i].normalize();
        dirs[i] *= (1.0 / d.dot(dirs[i]));
      }
      dirs[(i + 1)] += d;
    }
    return dirs;
  }

  void DrawRoadSegmentBorder(RoadSegment& segment) {
    std::vector<nacb::Vec2d> dirs = GetRoadSegmentNormals(segment);
    
    glLineWidth(2);
    glColor3f(1, 1, 1);
    glDepthMask(0);
    
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < int(segment.points.size()); ++i) {
      const nacb::Vec2d& p1 = segment.points[i];
      glVertex3f(p1.x - dirs[i].x * 0.45, p1.y - dirs[i].y * 0.45, -0.025);
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < int(segment.points.size()); ++i) {
      const nacb::Vec2d& p1 = segment.points[i];
      glVertex3f(p1.x + dirs[i].x * 0.45, p1.y + dirs[i].y * 0.45, -0.025);
    }
    glEnd();
    glDepthMask(1);

    glLineWidth(1);
  }

  void DrawRoadSegment(RoadSegment& segment) {
    glColor3f(1, 0, 0);

    if (false) {
      glBegin(GL_LINE_STRIP);
      for (const auto& co : segment.points) {
        glVertex2f(co.x, co.y);
      }
      glEnd();
    }

    std::vector<nacb::Vec2d> dirs = GetRoadSegmentNormals(segment);

    glColor3f(0.7, 0.7, 0.7);
    glDepthMask(0);    
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i < int(segment.points.size()); ++i) {
      const nacb::Vec2d& p1 = segment.points[i];
      glVertex2f(p1.x - dirs[i].x * 0.45, p1.y - dirs[i].y * 0.45);
      glVertex2f(p1.x + dirs[i].x * 0.45, p1.y + dirs[i].y * 0.45);
    }
    glEnd();
    glDepthMask(1);
    
  }

  void DrawRoadSegmentSpeed(const nacb::Quaternion& cquat,
                            RoadSegment& segment) {
    const double speed = segment.GetAverageSpeed();
    const nacb::Vec2d co = (segment.points[0] + segment.points[1]) * 0.5;
    char str[1024];
    snprintf(str, sizeof(str), "%3.2f", speed);

    glPushMatrix();
    glTranslatef(co.x, co.y - 0.05, 0);
    cquat.glRotate();

    glDisable(GL_TEXTURE_2D);
    glBegin(GL_LINES);
    glColor4f(1, 1, 1, 0.1);

    glVertex3f(0, 0, 0);
    glColor4f(1, 1, 1, 0.8);
    glVertex3f(0, 0, 0.8);
    glEnd();

    glEnable(GL_TEXTURE_2D);
    glTranslatef(0, 0, 1.0);
    ffont_.drawString(str, 0., 0.);
    
    glPopMatrix();
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

    wheel_mesh_.Bind(true);

    glColor3f(0.2, 0.2, 0.2);
    glPushMatrix();
    glTranslatef(0.85/4, -1.0/4.0, 0.4/4);
    r = 0.39 / 4.0 * 2 * M_PI;
    glRotatef(car.wheel_rot_ * 180.0 / M_PI, 0, 0, 1);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw(false);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.85/4,  1.0/4.0, 0.4/4);
    glRotatef(car.wheel_rot_ * 180.0 / M_PI, 0, 0, 1);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw(false);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-0.85/4,  -1.0/4.0, 0.4/4);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw(false);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-0.85/4,  1.0/4.0, 0.4/4);
    glRotatef(car.wheel_anim_ * 180.0 / M_PI, 0, 1, 0);
    wheel_mesh_.draw(false);
    glPopMatrix();

    wheel_mesh_.Bind(false);

    glDepthMask(0);
    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glColor3f(1, 1, 1);
    drop_shadow_mesh_.draw();
    glEnable(GL_LIGHTING);
    glDepthMask(1);

    glPopMatrix();
  }

  void DrawLevel(const nacb::Quaternion&  cquat,
                 Level& level,
                 std::vector<Car>& cars) {
    DrawGroundPlane();
    
    for (int i = 0; i < (int)level.intersections.size(); ++i) {
      DrawIntersection(level.intersections[i]);
    }
    
    for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
      DrawParkingLot(level.parking_lots[i]);
    }

    for (int i = 0; i < (int)level.road_segments.size(); ++i) {
      DrawRoadSegmentBorder(level.road_segments[i]);
    }
    for (int i = 0; i < (int)level.road_segments.size(); ++i) {
      DrawRoadSegment(level.road_segments[i]);
    }

    if (car_mesh_.empty()) {
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

    glPushMatrix();
    for (int i = 0; i < (int)level.road_segments.size(); ++i) {
      DrawRoadSegmentSpeed(cquat, level.road_segments[i]);
    }
    glPopMatrix();
    
    glDisable(GL_TEXTURE_2D);
  }

protected:
  VBOMesh car_mesh_;
  VBOMesh wheel_mesh_;
  VBOMesh drop_shadow_mesh_;
  nappear::Mesh stop_sign_mesh_;
  nappear::Mesh lights_mesh_;
  nappear::Mesh light_mesh_;
  VBOMesh tail_light_mesh_;
  GLuint tex_ = 0;
};


struct PosOrder {
  bool operator()(Car* a, Car* b) {
    return a->pos().x + a->pos().y < b->pos().x + b->pos().y;
  }
};

class LevelWindow: public GLWindow {
public:
  LevelWindow(Level& level, bool benchmark=false) : GLWindow(1280, 720), level_(level), planner_(level)  {
    glewInit();

    cpos.y = -50;
    cpos.z = 40;
    farPlane = 1000;
    if (!benchmark) {
      setRefreshRate(60);
    }

    if (false) {
      for (int i = 0; i < 7; ++i) {
        cars_.push_back(Car(level, planner_, i, (i) % 7, ((i) + 1) % 7));
      }
    } else {
      for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
        for (int j = 0; j < (int)level.parking_lots[i].parking_spots.size(); ++j) {
          int other_lot = (i + 1 + rand()) % level.parking_lots.size();
          if (other_lot == i) other_lot = (i + 1) % level.parking_lots.size();
          cars_.push_back(Car(level, planner_, cars_.size(), i, other_lot));
        }
      }
    }
    for (auto& car : cars_) {
      cars_p_.push_back(&car);
    }
    glClearColor(0.5, 0.5, 0.7, 1.0);
    level_renderer_.Init();
  }

  bool keyboard(unsigned char c, int x, int y) {
    GLWindow::keyboard(c, x, y);
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

    nacb::Quaternion q = cquat;
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

      
      q = nacb::Quaternion::rod(nacb::Vec3d(15.0 * M_PI / 180, 0, 0)) *
        nacb::Quaternion::rod(nacb::Vec3d(-M_PI/2, 0, 0)) *
        nacb::Quaternion::rod(nacb::Vec3d(0, 0, -r + M_PI/2));
      q = q.conj();
    }
    level_renderer_.DrawLevel(q, level_, cars_);
  }
  void Step(double dt) {
    /*
    absl::flat_hash_map<nacb::Vec2d, std::vector<Car*>, PointHash, PointEqual> cars_by_point;
    for (auto& car: cars_) {
      cars_by_point[car.pos()].push_back(&car);
      cars_by_point[car.pos() - nacb::Vec2d(1, 0)].push_back(&car);
      cars_by_point[car.pos() - nacb::Vec2d(0, 1)].push_back(&car);
      cars_by_point[car.pos() + nacb::Vec2d(1, 0)].push_back(&car);
      cars_by_point[car.pos() + nacb::Vec2d(0, 1)].push_back(&car);
      cars_by_point[car.pos() + nacb::Vec2d(1, 1)].push_back(&car);
    }
    double cc_size = 0;
    */
    std::sort(cars_p_.begin(), cars_p_.end(), PosOrder());
    std::vector<Car*>::iterator low = cars_p_.begin();
    std::vector<Car*>::iterator high = cars_p_.begin();
    double dist = sqrt(2) * 4.5;
    for (auto& car: cars_p_) {
      while ((*low)->pos().x + (*low)->pos().y + dist < car->pos().x + car->pos().y) {
        low++;
      }
      for ( ; high != cars_p_.end() && (car->pos().x + car->pos().y > (*high)->pos().x + (*high)->pos().y - dist); ++high)
        ;
      car->Step(low, high, t_, dt);
    }
    for (auto& isect: level_.intersections) {
      isect.Step(dt);
    }
    t_ += dt;
  }

  void refresh() {
    static nacb::Timer timer;
    double dt = (double)timer;
    timer.reset();
    if (dt > 0) {
      Step(dt);
    }
    GLWindow::refresh();
  }
  double t_ = 0;
  Level& level_;
  plan::Planner planner_;
  std::vector<Car> cars_;
  std::vector<Car*> cars_p_;
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
  if (FLAGS_benchmark == "plan") {
    nacb::Timer timer;
    int num_times = 0;
    int plan_size = 0;
    plan::Planner planner(level);

    for (int i = 0; i < 100; ++i) {
      for (int i = 0; i < (int)level.parking_lots.size(); ++i) {
        for (int j = 0; j < (int)level.parking_lots.size(); ++j) {
          if (i == j) continue;
          auto plan = planner.Plan(level.parking_lots[i],
                                   (i  + j) % level.parking_lots[i].parking_spots.size(),
                                   level.parking_lots[j],
                                   (i  + j) % level.parking_lots[j].parking_spots.size());
          plan_size += plan[1].segments.size();
          num_times++;
        }
      }
      if (double(timer) > 5) break;
    }
    LOG(INFO) << "benchmark_plan=" << double(num_times) / double(timer) << " avg_plan_size="
              << double(plan_size) / num_times << " num_times=" << num_times;
    return 0;
  }
  LevelWindow level_window(level, FLAGS_benchmark == "render");

  if (FLAGS_benchmark == "render") {
    nacb::Timer timer;
    const int num_times = 1000;
    for (int i = 0; i < num_times; ++i) {
      level_window.drawScene();
    }
    LOG(INFO) << "benchmark_render=" << double(num_times) / double(timer);
    return 0;
  } else if (FLAGS_benchmark == "sim") {
    nacb::Timer timer;
    const int num_times = 100;
    for (int i = 0; i < num_times; ++i) {
      for (int j = 0; j < 60; ++j) {
        level_window.Step(1.0 / 60.0);
      }
    }
    LOG(INFO) << "sim_render=" << double(num_times) / double(timer);
    return 0;
  }

  
  level_window.loop(1);
  return 0;
}
