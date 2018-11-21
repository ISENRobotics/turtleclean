/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "turtleclean/turtle_frame.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtleclean
{

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
// , path_image_(1039,829, QImage::Format_ARGB32)
, path_image_(QImage(QString((ros::package::getPath("turtleclean") + "/images/chambre.png").c_str())))
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
{
  setFixedSize(1039,829);
  setWindowTitle("turtleclean");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_.setParam("background_r", DEFAULT_BG_R);
  nh_.setParam("background_g", DEFAULT_BG_G);
  nh_.setParam("background_b", DEFAULT_BG_B);

  QVector<QString> turtles;
  turtles.append("box-turtle.png");
  turtles.append("robot-turtle.png");
  turtles.append("sea-turtle.png");
  turtles.append("diamondback.png");
  turtles.append("electric.png");
  turtles.append("fuerte.png");
  turtles.append("groovy.png");
  turtles.append("hydro.svg");
  turtles.append("indigo.svg");
  turtles.append("jade.png");
  turtles.append("kinetic.png");

  QString images_path = (ros::package::getPath("turtleclean") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();

  clear();

  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

  ROS_INFO("Starting turtleclean with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTurtle("", width_in_meters_ / 4.0, height_in_meters_ / 4.0, 0);

  // spawn all available turtle types
  if(false)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(turtleclean::Spawn::Request& req, turtleclean::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::killCallback(turtleclean::Kill::Request& req, turtleclean::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  TurtlePtr t(new Turtle(ros::NodeHandle(real_name), turtle_images_[index], QPointF(x, height_in_meters_ - y), angle));
  turtles_[real_name] = t;
  update();

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  nh_.param("background_r", r, r);
  nh_.param("background_g", g, g);
  nh_.param("background_b", b, b);

  // path_image_.fill(qRgb(r, g, b));
  // path_image_ = QImage(QString((ros::package::getPath("turtleclean") + "/images/chambre.png").c_str())));
  update();
}

void TurtleFrame::onUpdate()
{
  ros::spinOnce();

  updateTurtles();

  if (!ros::ok())
  {
    close();
  }
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);
  // QString background_path = (ros::package::getPath("turtleclean") + "/images/chambre.png").c_str();
  // painter.drawImage(QPoint(0, 0), QImage(background_path));

  // TODO : START CA

  // painter.setBrush(Qt::NoBrush);
  // painter.setPen(Qt::red);
  // painter.drawRect(0, 0, QPointF(x, height_in_meters_ - y));
  // painter.drawPoint(QPointF(1, height_in_meters_ + 1));
  // painter.drawPoint(QPointF(10, height_in_meters_ + 10));
  // painter.setPen(Qt::blue);
  // //qreal x1 = 0 *1039/23;
  // //qreal x2 = 23 * 1039 / 23;
  // //qreal y1 = 829-(0 * 829 / 18);
  // //qreal y2 = 829-(18 * 829 / 18);
  // painter.drawLine(QPointF(x1,y1),QPointF(x2,y2));
  //Console
  QVector<qreal> px;
  QVector<qreal> py;
  px.append(4.8);
  py.append(1);
  px.append(4.8);
  py.append(2.65);
  px.append(17.75);
  py.append(2.65);
  px.append(17.75);
  py.append(1);

  QPolygonF pPolygon;
  for (uint k=0;k<px.size();k++)
    pPolygon << QPointF(px.at(k) * 1039 / 23, 829 - (py.at(k) * 829 / 18));
  painter.setBrush(QColor(128, 128, 128, 100));
  painter.drawPolygon(pPolygon, Qt::WindingFill);
// Lit
px.clear();
py.clear();
px.append(6.6);
py.append(5.15);

px.append(6.6);
py.append(16.0);

px.append(6.1);
py.append(16.0);

px.append(6.1);
py.append(16.75);

px.append(15.95);
py.append(16.75);
px.append(16.45);
py.append(16.75);
px.append(16.45);
py.append(16.0);
px.append(15.95);
py.append(16.0);

px.append(15.95);
py.append(5.15);
pPolygon.clear();
for (uint k = 0; k < px.size(); k++)
        pPolygon
    << QPointF(px.at(k) * 1039 / 23, 829 - (py.at(k) * 829 / 18));
painter.drawPolygon(pPolygon, Qt::WindingFill);
// Chevet Gauche
px.clear();
py.clear();
px.append(2.25);
py.append(13.65);
px.append(2.25);
py.append(16.05);
px.append(5.7);
py.append(16.05);
px.append(5.7);
py.append(13.65);
pPolygon.clear();
for (uint k = 0; k < px.size(); k++)
  pPolygon
      << QPointF(px.at(k) * 1039 / 23, 829 - (py.at(k) * 829 / 18));
painter.drawPolygon(pPolygon, Qt::WindingFill);
// Chevet Droit
px.clear();
py.clear();
px.append(14.6+2.25);
py.append(13.65);
px.append(14.6+2.25);
py.append(16.05);
px.append(14.6+5.7);
py.append(16.05);
px.append(14.6+5.7);
py.append(13.65);
pPolygon.clear();
for (uint k = 0; k < px.size(); k++)
  pPolygon
      << QPointF(px.at(k) * 1039 / 23, 829 - (py.at(k) * 829 / 18));
painter.drawPolygon(pPolygon, Qt::WindingFill);

// TODO : end CA
M_Turtle::iterator it = turtles_.begin();
   TODO : M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtleclean.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtleclean.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}
}
