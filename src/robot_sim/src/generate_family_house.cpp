// Self-contained Gazebo Classic family house; no external model downloads.
// Geometry, route and validation metadata share a single source of truth.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

struct Part {
  std::string name, color;
  double x, y, z, sx, sy, sz;
  bool collision;
};
struct Point { double x, y; };
struct Room { std::string name; double x0, x1, y0, y1; Point goal; };
class House {
 public:
  std::vector<Part> parts;
  std::vector<Room> rooms;
  std::vector<Point> route;
  void box(const std::string &name, double x, double y, double z,
           double sx, double sy, double sz, const std::string &color,
           bool collision = true) {
    parts.push_back({name, color, x, y, z, sx, sy, sz, collision});
  }
  void wall_x(const std::string &name, double x0, double x1, double y) {
    box(name, (x0+x1)/2, y, 1.2, x1-x0, .16, 2.4, ".82 .83 .78 1");
  }
  void wall_y(const std::string &name, double x, double y0, double y1) {
    box(name, x, (y0+y1)/2, 1.2, .16, y1-y0, 2.4, ".82 .83 .78 1");
  }
  void table(const std::string &name, double x, double y, double sx, double sy,
             double h, const std::string &color) {
    box(name+"_top", x,y,h-.04,sx,sy,.08,color);
    for (int i : {-1,1}) for (int j : {-1,1}) {
      box(name+"_leg_"+std::to_string(i)+"_"+std::to_string(j),
          x+i*(sx/2-.09),y+j*(sy/2-.09),(h-.08)/2,.09,.09,h-.08,".23 .17 .12 1");
    }
  }
  void chair(const std::string &name, double x, double y) {
    table(name,x,y,.46,.46,.46,".57 .39 .23 1");
    box(name+"_back",x,y-.20,.68,.46,.07,.5,".57 .39 .23 1");
  }
  void bed(const std::string &name, double x, double y, double sx, const std::string &color) {
    box(name+"_base",x,y,.18,sx,2.0,.36,".38 .25 .16 1");
    box(name+"_duvet",x,y-.1,.43,sx,1.8,.14,color);
    box(name+"_headboard",x,y+.99,.65,sx,.10,1.3,".38 .25 .16 1");
    box(name+"_pillow",x,y+.62,.54,sx*.74,.42,.12,".94 .92 .84 1");
  }
  bool free(double x, double y, double radius) const {
    for (const auto &p : parts) {
      // Baobao is 0.8 m tall: tabletops/chair seats also obstruct the robot.
      // Keep physical height in the clearance test rather than using the
      // reference short car's 0.27 m envelope.
      if (!p.collision || p.z+p.sz/2 <= .025 || p.z-p.sz/2 >= .85) continue;
      const double dx=std::max(std::abs(x-p.x)-p.sx/2,0.0);
      const double dy=std::max(std::abs(y-p.y)-p.sy/2,0.0);
      if (std::hypot(dx,dy) <= radius) return false;
    }
    return true;
  }
  void validate() const {
    if (!free(0,0,.35)) throw std::runtime_error("Spawn blocked");
    Point prev{0,0};
    for (const auto &next : route) {
      const int n=std::max(1,static_cast<int>(std::ceil(std::hypot(next.x-prev.x,next.y-prev.y)/.025)));
      for(int i=0;i<=n;++i) {
        const double t=static_cast<double>(i)/n;
        if(!free(prev.x+t*(next.x-prev.x),prev.y+t*(next.y-prev.y),.38))
          throw std::runtime_error("Route clearance below 0.38 m near "+std::to_string(next.x)+","+std::to_string(next.y));
      }
      prev=next;
    }
    // Independent connected-space check with a 0.35 m inflated footprint.
    constexpr double step=.05;
    constexpr int nx=321, ny=241;
    std::vector<unsigned char> seen(nx*ny,0);
    auto index=[](int x,int y){return y*nx+x;};
    std::queue<std::pair<int,int>> q;
    q.push({160,120}); seen[index(160,120)]=1;
    while(!q.empty()) {
      const auto [x,y]=q.front(); q.pop();
      for(const auto &[dx,dy] : std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}) {
        const int a=x+dx,b=y+dy;
        if(a<0 || a>=nx || b<0 || b>=ny || seen[index(a,b)] || !free(-8+a*step,-6+b*step,.35)) continue;
        seen[index(a,b)]=1; q.push({a,b});
      }
    }
    for(const auto &r:rooms) {
      const int x=std::lround((r.goal.x+8)/step), y=std::lround((r.goal.y+6)/step);
      if(!seen[index(x,y)]) throw std::runtime_error("Room unreachable: "+r.name);
    }
  }
  void write(const std::filesystem::path &root) const {
    validate();
    std::filesystem::create_directories(root/"worlds");
    std::filesystem::create_directories(root/"config");
    std::ofstream sdf(root/"worlds/family_house.world");
    sdf << std::setprecision(8) << "<?xml version=\"1.0\"?>\n<sdf version=\"1.6\"><world name=\"family_house\">\n"
        << "<gravity>0 0 -9.81</gravity>\n<physics name=\"ode\" type=\"ode\"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate><real_time_factor>1</real_time_factor></physics>\n"
        << "<scene><ambient>0.65 0.65 0.65 1</ambient><background>0.75 0.82 0.9 1</background><shadows>true</shadows></scene>\n"
        << "<light name=\"sun\" type=\"directional\"><cast_shadows>true</cast_shadows><pose>0 0 12 0 0 0</pose><diffuse>0.85 0.85 0.82 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 -0.4 -1</direction></light>\n"
        << "<gui fullscreen=\"0\"><camera name=\"house_view\"><pose>0 -17 19 0 0.85 1.57079632679</pose><view_controller>orbit</view_controller></camera></gui>\n";
    // One static link keeps rendering/physics inexpensive; each part still has
    // a matching collision unless explicitly marked as floor decoration.
    sdf << "<model name=\"family_house\"><static>true</static><link name=\"structure\">\n";
    for(const auto &p:parts) {
      const std::string pose=std::to_string(p.x)+" "+std::to_string(p.y)+" "+std::to_string(p.z)+" 0 0 0";
      const std::string size=std::to_string(p.sx)+" "+std::to_string(p.sy)+" "+std::to_string(p.sz);
      if(p.collision) sdf << "<collision name=\""<<p.name<<"_collision\"><pose>"<<pose<<"</pose><geometry><box><size>"<<size<<"</size></box></geometry></collision>\n";
      sdf << "<visual name=\""<<p.name<<"\"><pose>"<<pose<<"</pose><geometry><box><size>"<<size<<"</size></box></geometry><material><ambient>"<<p.color<<"</ambient><diffuse>"<<p.color<<"</diffuse></material></visual>\n";
    }
    sdf << "</link></model></world></sdf>\n";
    std::ofstream json(root/"worlds/family_house.layout.json");
    json << "{\n\"size_m\":[16,12],\"door_clear_width_m\":1.2,\"corridor_clear_width_m\":1.84,\"robot_radius_m\":0.35,\"robot_height_m\":0.85,\"route_clearance_m\":0.38,\n\"rooms\":[";
    for(size_t i=0;i<rooms.size();++i) {
      const auto &r=rooms[i]; if(i) json<<",";
      json << "{\"name\":\""<<r.name<<"\",\"bounds\":["<<r.x0<<","<<r.x1<<","<<r.y0<<","<<r.y1<<"],\"goal\":["<<r.goal.x<<","<<r.goal.y<<"]}";
    }
    json << "],\n\"parts\":[";
    for(size_t i=0;i<parts.size();++i) {
      const auto &p=parts[i]; if(i) json<<",";
      json << "{\"name\":\""<<p.name<<"\",\"pose\":["<<p.x<<","<<p.y<<","<<p.z<<"],\"size\":["<<p.sx<<","<<p.sy<<","<<p.sz<<"],\"color\":\""<<p.color<<"\",\"collision\":"<<(p.collision?"true":"false")<<"}";
    }
    json << "],\n\"route\":[";
    for(size_t i=0;i<route.size();++i) {if(i)json<<",";json<<"["<<route[i].x<<","<<route[i].y<<"]";}
    json << "]}\n";
    std::ofstream yaml(root/"config/family_house_mapping_route.yaml");
    // ROS 2 parameter arrays must be homogeneous: emit 0.000, never integer 0.
    yaml << std::fixed << std::setprecision(3);
    yaml << "# Generated from generate_family_house.cpp; 0.38 m geometric clearance.\n# Scene-specific waypoint tour, not autonomous exploration.\nmapping_route:\n  ros__parameters:\n    use_sim_time: true\n    speed: 0.30\n    waypoints: [";
    for(size_t i=0;i<route.size();++i) {if(i)yaml<<", ";yaml<<route[i].x<<", "<<route[i].y;}
    yaml << "]\n";
    if(!sdf || !json || !yaml) throw std::runtime_error("Failed to write house assets");
    std::cout << "Generated "<<parts.size()<<" parts, "<<rooms.size()<<" reachable rooms, "<<route.size()<<" validated waypoints in "<<root<<"\n";
  }
};

int main(int argc, char **argv) {
  try {
    if(argc!=2) throw std::runtime_error("Usage: generate_family_house /path/to/robot_sim (replaces generated family_house assets)");
    House h;
    h.box("floor",0,0,-.10,16.3,12.3,.20,".64 .58 .47 1");
    h.wall_x("outer_north",-8.08,8.08,6);
    h.wall_x("outer_south",-8.08,8.08,-6);
    h.wall_y("outer_west",-8,-6,6);
    h.wall_y("outer_east",8,-6,6);
    // Six 1.20 m clear corridor doorways, lintels start above 2 m.
    for(int side:{-1,1}) {
      const std::string s=side<0?"south":"north";
      double left=-8;
      int i=0;
      for(double center:{-5.3,0.0,5.3}) {
        h.wall_x(s+"_corridor_"+std::to_string(i),left,center-.6,side);
        h.box(s+"_door_lintel_"+std::to_string(i),center,side,2.25,1.2,.16,.3,".90 .86 .75 1");
        left=center+.6; ++i;
      }
      h.wall_x(s+"_corridor_end",left,8,side);
      // Connecting doors form loops, useful for SLAM closure/navigation.
      for(int k:{-1,1}) {
        const double x=k*2.6;
        const std::string n=s+"_partition_"+std::to_string(k);
        if(side>0) {h.wall_y(n+"a",x,1,2.7);h.wall_y(n+"b",x,3.9,6);}
        else {h.wall_y(n+"a",x,-6,-3.9);h.wall_y(n+"b",x,-2.7,-1);}
        h.box(n+"_lintel",x,side*3.3,2.25,.16,1.2,.3,".90 .86 .75 1");
      }
    }
    h.rooms={{"Main bedroom",-8,-2.6,1,6,{-5.3,2.2}},
      {"Study",-2.6,2.6,1,6,{0,2.2}}, {"Child bedroom",2.6,8,1,6,{5.3,2.2}},
      {"Kitchen and dining",-8,-2.6,-6,-1,{-5.3,-2.2}},
      {"Living room",-2.6,2.6,-6,-1,{0,-2.2}},
      {"Bathroom and laundry",2.6,8,-6,-1,{5.3,-2.2}}};
    for(size_t i=0;i<h.rooms.size();++i) {
      const auto &r=h.rooms[i];
      const std::vector<std::string> colors={".76 .69 .57 1",".65 .67 .56 1",".64 .73 .77 1",".72 .72 .68 1",".78 .70 .59 1",".66 .78 .78 1"};
      h.box("room_floor_"+std::to_string(i),(r.x0+r.x1)/2,(r.y0+r.y1)/2,.002,r.x1-r.x0-.16,r.y1-r.y0-.16,.004,colors[i],false);
    }
    h.box("corridor_floor",0,0,.003,15.84,1.84,.006,".86 .82 .71 1",false);
    h.bed("master_bed",-6.5,4.8,1.8,".35 .49 .65 1");
    h.box("master_wardrobe",-3.35,5.35,1.0,1.15,1.1,2,".45 .30 .20 1");
    h.box("master_nightstand",-7.55,4.9,.25,.45,.6,.5,".50 .34 .22 1");
    h.table("study_desk",.2,5.35,1.8,.8,.75,".65 .45 .25 1");
    h.chair("study_chair",.2,4.45);
    h.box("study_bookshelf",-1.95,5.45,1.0,.8,.7,2,".40 .28 .17 1");
    h.box("study_monitor",.2,5.45,1.0,.65,.10,.4,".12 .15 .17 1");
    h.box("study_cabinet",1.95,5.25,.55,.8,1.1,1.1,".62 .47 .29 1");
    h.bed("child_bed",6.6,4.8,1.2,".71 .46 .35 1");
    h.box("toy_shelf",3.4,5.5,.5,1.2,.75,1,".75 .61 .32 1");
    h.table("child_desk",7.4,2,.7,1.2,.7,".68 .50 .28 1");
    h.box("child_rug",5,4.5,.006,1.2,1.4,.006,".40 .58 .62 1",false);
    h.box("kitchen_counter_south",-5.65,-5.55,.45,3.8,.75,.9,".68 .72 .65 1");
    h.box("kitchen_counter_west",-7.55,-4.4,.45,.75,1.55,.9,".68 .72 .65 1");
    h.box("fridge",-7.5,-1.8,.95,.8,1.0,1.9,".82 .85 .85 1");
    h.box("sink_inset",-5.6,-5.55,.91,.65,.48,.03,".35 .43 .45 1");
    h.box("stove",-4.3,-5.55,.92,.65,.55,.04,".16 .18 .19 1");
    h.table("dining_table",-5.5,-4.1,1.5,.8,.75,".63 .43 .23 1");
    h.chair("dining_chair_a",-6.55,-4.1);
    h.chair("dining_chair_b",-4.45,-4.1);
    h.box("sofa_base",-.3,-5.15,.22,2.4,.85,.44,".31 .43 .48 1");
    h.box("sofa_back",-.3,-5.55,.62,2.4,.18,.8,".26 .37 .41 1");
    h.box("sofa_arm_left",-1.45,-5.15,.48,.18,.85,.52,".26 .37 .41 1");
    h.box("sofa_arm_right",.85,-5.15,.48,.18,.85,.52,".26 .37 .41 1");
    h.table("coffee_table",-.3,-4.0,1.2,.6,.42,".59 .39 .22 1");
    h.box("tv_console",2.0,-4.8,.25,.6,1.6,.5,".44 .31 .21 1");
    h.box("tv_screen",2.0,-4.8,.95,.10,1.25,.75,".09 .12 .15 1");
    h.box("living_rug",-.3,-4.2,.006,3.2,2.5,.006,".70 .63 .50 1",false);
    // Tub has a real hollow center instead of a solid bounding box.
    h.box("tub_bottom",6.5,-5.15,.10,2,.95,.20,".90 .93 .92 1");
    h.box("tub_north",6.5,-4.72,.33,2,.10,.46,".90 .93 .92 1");
    h.box("tub_south",6.5,-5.58,.33,2,.10,.46,".90 .93 .92 1");
    h.box("tub_west",5.55,-5.15,.33,.10,.8,.46,".90 .93 .92 1");
    h.box("tub_east",7.45,-5.15,.33,.10,.8,.46,".90 .93 .92 1");
    h.box("vanity",3.65,-5.5,.43,1.35,.7,.86,".74 .80 .79 1");
    h.box("vanity_basin",3.65,-5.5,.9,.6,.5,.08,".92 .95 .95 1");
    h.box("washing_machine",7.4,-1.85,.43,.8,.85,.86,".83 .86 .87 1");
    h.box("toilet_base",7.25,-3.5,.2,.5,.65,.4,".91 .94 .93 1");
    h.box("toilet_tank",7.55,-3.5,.55,.22,.55,.65,".91 .94 .93 1");
    // Windows are opaque blue inset panels high on walls; no collision holes.
    for(int side:{-1,1}) for(int i:{-1,0,1}) {
      h.box("window_"+std::to_string(side)+"_"+std::to_string(i),i*5.3,side*5.91,1.55,1.7,.025,.8,".36 .60 .70 1",false);
    }
    // Closed loops, all straight legs validated against the actual low geometry.
    h.route={{-5.3,0},{-5.3,2.2},{-5.3,3.3},{0,3.3},{0,2.2},{0,0},
      {5.3,0},{5.3,2.2},{5.3,3.3},{0,3.3},{0,0},
      // Turn before the dining tabletop, then line up with the side door.
      // The tilted 3D lidar sees this tabletop; the old low 2D lidar did not.
      {-5.3,0},{-5.3,-2.2},{-5.3,-3.0},{-3.3,-3.0},{-3.3,-3.3},{0,-3.3},{0,-2.2},{0,0},
      {5.3,0},{5.3,-2.2},{5.3,-3.3},{0,-3.3},{0,0}};
    h.write(argv[1]);
    return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<"\n";return 1;}
}
