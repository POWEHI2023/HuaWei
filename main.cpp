
/**
 * 需要解决的问题
 * 
 * 1. 制定泊点的路径/ 送货点的路径
 * 2. 船路径加锁/ 
 * 3. 并发
 * 4. 控制船和机器人的数量
 * 
 * 正在实现的东西
 * 
 * 
*/
#pragma warning(disable:4996)

#include <iostream>
#include <memory>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <vector>
#include <queue>
#include <tuple>
#include <climits>
#include <typeinfo>
#include <string>
#include <numeric>
#include <set>
#include <cassert>
#include <functional>
#include <thread>
#include <exception>
#include <cstring>
#include <mutex>
#include <unordered_set>
#include <stack>



#define N 200
#define BOAT_PRICE 8000
#define ROBOT_PRICE 2000

#define MAX_BOAT_LIMIT 10     // BOAT_LIMIT不应该超过MAX_BOAT_LIMIT

#define ROBOT_LIMIT 15        // 设置一个最大量
#define BOAT_LIMIT 1         // 设置一个最大量

#define INIT_ROBOT_NUM 2      // 初始买几个机器人
#define INIT_BOAT_NUM 1       // 初始买几个船

#define ROUTER_LIMIT_PER_FRAME 1        // 每帧最多找几次路
#define CMP_TARGET_NUM        10         // 每次比较多少个货物确定要找的货物

#define display(msg, ...) ({ fprintf(stderr, #msg, ##__VA_ARGS__); })
#define CHECK_OK() ({ char okk[100]; scanf("%s", okk); })

typedef std::tuple<int, int> Position;
enum Direction {RIGHT = 0, LEFT, UP, DOWN, POINT};
constexpr int CLOCKWISE = 1 << 5;
enum Rotation {CLOCK = CLOCKWISE, ANTI, SHIP};

#define as_key(x, y) (((x) << 10) | (y))
#define as_position(key) ({ {(key) >> 10, (key) & 0x3FF} })

/**
 * 提前声明类型方便后续访问
*/

typedef std::vector<int> Path__;
class Path;

enum TraceType { GOOD, BERTH, None };

// 用来构造路径
Path __trace_back(int const map[N][N], int x, int y)  ;

/**
 * 路径指针，用来移动
 * tar参数为目标位置
 * TraceType为寻路的目标类型
 * distance为路径距离
*/
class Path: public Path__ {
public:
          template <typename... TArgs>
          Path(TArgs... args): Path__(std::forward<TArgs>(args)...), cursor(-1), tar_x(0), tar_y(0), type_(None) {}
          inline int get_distance() const   { return cursor + 1; }
          inline bool done() const   { return cursor < 0; }
          inline int get_direction()   { return done() ? POINT : (*this)[cursor--]; }

          int tar_x, tar_y;
          TraceType type_;
          int tar_price = 0;
private:
          int cursor;      // 指针从后往前
          friend Path __trace_back(std::vector<std::vector<int>> &, int, int)  ;
          friend Path __trace_back_boat(std::vector< std::vector< std::vector<int> > > &direction_, int x, int y, int dir)  ;
};

void perfix_action()  ;          // 预备动作
void robot_action()  ;           // 机器人做出动作
void boat_action()  ;            // 船做出动作
void suffix_action()  ;

/**
 * 四个地图上的基本元素 [基本不会被修改]
*/
namespace BaseElem {
          struct Berth__ {
                    int id, x, y;
                    int loading_speed;
          };
          struct Good__ {
                    int x, y;  // 货物的位置坐标
                    int price; // 货物的价值
          };
          struct Robot__ {
                    int id, x, y, goods_num;

                    inline void move(int dir) const   { printf("move %d %d\n", id, dir); }
                    inline void get() const   { printf("get %d\n", id); }
                    inline void pull() const   { printf("pull %d\n", id); }
          };
          struct Boat__ {
                    int id, x, y, dir;
                    int goods_num, status;
                    // status.0 == 正常/ status.1 == 恢复/ status.2 == 装载/ dir与机器人Direction一致

                    inline void dept() const   { printf("dept %d\n", id); }
                    inline void berth() const   { printf("berth %d\n", id); }
                    inline void rot(int rot) const   { printf("rot %d %d\n", id, rot - CLOCKWISE); }
                    inline void ship() const   { printf("ship %d\n", id); }
          };
}

/**
 * 扩充地图上的基本元素
*/

// TODO: 
struct Berth: public BaseElem::Berth__ {
          int crt_num = 0;
          // std::mutex mtx;

          // 是否是空闲的，没船占用的
          bool free_ = true;
};
struct Good: public BaseElem::Good__ {
          int purchase_rank = 0;
          int live = 1000;              // 全部存活时间

          Good(): Good__{0, 0, 0}, purchase_rank(-1), live(0) {}
          Good(int x, int y, int price, int purchase_rank, int live): Good__{x, y, price},
           purchase_rank(purchase_rank), live(live) {}
};

struct Robot: public BaseElem::Robot__ {
          int trace_x = 0, trace_y = 0; // 追踪应该在的位置/ 如果x != trace_x || y != trace_y说明发生了碰撞
          bool collision = false;

          Path current_path;
          int current_price = 0;

          std::vector<int> stack_; // 用于移动过程中动态添加的路径

          inline void pull() const  ;
          inline void move(int)  ;
          inline void get()  ;
};

struct Boat: public BaseElem::Boat__ {
          int crt_num = 0;              // 当前载货
          bool can_leave = false;       // 是否载货后离开送货
                    // 送货路上是true/ 取货路上是false
          Berth *aim = nullptr;         // 目标泊点
          Path current_path;            // 待走路径

          int trace_x = 0, trace_y = 0, trace_dir = 0;
          bool collition = false;

          inline void move()  ;
          inline void set_aim(Berth *berth)   {
                    aim = berth;
                    berth->free_ = false;
          }
          inline void release_aim()   {
                    aim->free_ = true;
                    aim = nullptr;
          }
};

/**
 * 扩展的用于存储`机器人`与`泊点`这种包含x、y坐标信息的类型 [基本不会被修改]
*/
namespace BaseElem {
          typedef bool (*cmp) (int x, int y, const Berth &b);
          template <typename T>
          class IfStor__ {
          public:
                    inline bool find(int x, int y, const cmp fn = nullptr)   {
                              return static_cast<T *>(this)->find_(x, y, fn);
                    }
          };

          typedef std::vector<Berth> BerthStor__;
          typedef std::unordered_map<int, Good> GoodStor__;

          class BerthStor: /*public BerthStor__,*/ public IfStor__<BerthStor> {
          public:
                    template <typename... TArgs> explicit BerthStor(TArgs... args): entity_( BerthStor__(std::forward<TArgs>(args)...) ) {}

                    inline decltype(auto) iter_find(int x, int y, const std::function<bool(int,int,Berth &)> &cmp = [](int x,int y, Berth &berth) {
                              return x == berth.x && y == berth.y;
                    }) {
                              display(ITER::: tar_x %d tar_y %d\n, x, y);
                              for (auto iter = entity_.begin(); iter != entity_.end(); ++iter) {
                                        display(CHECK::: berth %d x %d y %d\n, (*iter).id, (*iter).x, (*iter).y);
                                        if (cmp(x, y, *iter)) return iter;
                              }
                              return entity_.end();
                    }
                    inline decltype(auto) iter_find(int x, int y)   {
                              for (auto iter = entity_.begin(); iter != entity_.end(); ++iter)
                              if ((*iter).x == x && (*iter).y == y) return iter;
                              return entity_.end();
                    }
                    // 接口方法
                    inline bool find_(int x, int y, const cmp fn = nullptr)   {
                              if (fn == nullptr) {
                                        for (auto iter = entity_.begin(); iter != entity_.end(); ++iter)
                                        if ((*iter).x == x && (*iter).y == y) return true;
                                        return false;
                              } else {
                                        for (auto iter = entity_.begin(); iter != entity_.end(); ++iter)
                                        if (fn(x, y, *iter)) return true;
                                        return false;
                              }
                    }

                    inline decltype(auto) operator[] (size_t index) { return entity_[index]; }
                    inline decltype(auto) begin() { return entity_.begin(); }
                    inline decltype(auto) end() { return entity_.end(); }
                    inline decltype(auto) size() { return entity_.size(); }

                    template <typename... TArgs> decltype(auto) resize(TArgs... args) { entity_.resize(std::forward<TArgs>(args)...); }
                    template <typename... TArgs> decltype(auto) emplace_back(TArgs... args) { entity_.emplace_back(std::forward<TArgs>(args)...); }
                    template <typename... TArgs> decltype(auto) push_back(TArgs... args) { entity_.push_back(std::forward<TArgs>(args)...); }
          private:
                    BerthStor__ entity_;
          };

          class GoodStor: /*public GoodStor__,*/ public IfStor__<GoodStor> {
          public:
                    template <typename... TArgs>
                    explicit GoodStor(TArgs... args): entity_( GoodStor__(std::forward<TArgs>(args)...) ) {}

                    // 货物就只有一个x、y，所以直接返回bool
                    inline bool find_(int key)   { return (entity_.find(key) != entity_.end()); }
                              // 接口方法
                    inline bool find_(int x, int y, const cmp fn = nullptr)   { return find_(as_key(x, y)); }
                    inline decltype(auto) get(int key)   { return entity_[key]; }
                    inline decltype(auto) get(int x, int y)   { return entity_[as_key(x, y)]; }

                    inline decltype(auto) operator[] (size_t index) { return entity_[index]; }
                    inline decltype(auto) begin() { return entity_.begin(); }
                    inline decltype(auto) end() { return entity_.end(); }

                    inline decltype(auto) size() { return entity_.size(); }
                    template <typename... TArgs> inline decltype(auto) erase(TArgs... args) { return entity_.erase(std::forward<TArgs>(args)...); }
          private:
                    GoodStor__ entity_;
          };
}
/** 
 * 统一的接口 Storage，包含GoodStorage和BerthStorage
 * 只包含返回bool的find用来确认是否到达某位置
*/
template <typename T>
using Storage = BaseElem::IfStor__<T>;
// typedef IfStor__ Storage;

// using GoodStor = std::unordered_map<int, std::tuple<int, int>>;
// using BerthStor = std::vector<Berth>;
using GoodStor = BaseElem::GoodStor;
using BerthStor = BaseElem::BerthStor;

/**
 * 信息分类与初始化
 *        1. Base: 存储地图与泊点信息(帧无关)
 *        2. Frame: 存储每帧都会更新的机器人、货船与货物信息(帧相关)
 * [基本不会被修改]
*/
namespace BaseElem {
          typedef std::vector<std::tuple<int, int>> PurchasePoint__;
          template <typename T> class PurchasePoint;

          // Base
          struct Base {
                    static inline void init()   {
                              grid.resize(N);
                              for (int i = 0; i < N; ++i) std::cin >> grid[i];

                              Base::ProcessMap();

                              scanf("%d", &berth_num);
                              berths.resize(berth_num);
                              for (int i = 0; i < berth_num; ++i) {
                                        int id; scanf("%d", &id);

                                        display(BERTH::: %d -> id %d\n, i, id);
                                        scanf("%d%d%d", &berths[id].x, &berths[id].y, &berths[id].loading_speed);
                                        berths[id].id = i;
                              }
                              scanf("%d", &boat_capacity);
                              
                              CHECK_OK(); printf("OK\n"); fflush(stdout);
                    }
                    static inline void dealloc() {}
                    
                    static BerthStor berths;
                    static int boat_capacity;
                    static std::vector<std::string> grid;

                    static int robot_num;
                    static int boat_num;
                    static int berth_num;
                    static int goods_num;

                    static PurchasePoint<Robot> robot_purchase_point;
                    static PurchasePoint<Boat> boat_purchase_point;
          private:
                    static void ProcessMap()  ;
          };

          // Frame
          struct Frame {
                    static int id, money;
                    static GoodStor goods;
                    static std::vector<Robot> robots;
                    static std::vector<Boat> boats;

                    static int gap_id;
                    
                    static inline void init()   {
                              Frame::robots.reserve(ROBOT_LIMIT);
                              Frame::boats.reserve(BOAT_LIMIT);
                    }
                    static inline int update()  ;

                    static inline void dealloc() {}
          };

          /**
           * Purchase部分
          */

          template <typename T>
          class PurchasePoint {
          public:
                    template <typename... TArgs>
                    explicit PurchasePoint(TArgs... args) {
                              entity_ = PurchasePoint__(std::forward<TArgs>(args)...);
                              entity_.reserve(10);
                              good_in_range.reserve(10);
                    }
                    
                    // make instance in purchase/ robot or boat
                    bool purchase(int purchase_id) const   {
                              if (purchase_id >= entity_.size()) {
                                        fprintf(stderr, "Purchae id out of boundary......\n");
                                        return false;
                              }

                              if constexpr (std::is_same_v<T, Robot>) {
                                        if (Frame::money < ROBOT_PRICE) return false;

                                        Base::robot_num++;
                                        Frame::money -= ROBOT_PRICE;   // 更新数据

                                        const auto [pur_x, pur_y] = entity_[purchase_id];
                                        printf("lbot %d %d\n", pur_x, pur_y);
                                        Frame::robots.push_back(Robot {
                                                  (int)Frame::robots.size(), pur_x, pur_y, 0,       // id/ x/ y/ goods_num
                                                  pur_x, pur_y                                      // trace_x/ trace_y
                                        });
                              } else {
                                        if (Frame::money < BOAT_PRICE) return false;

                                        Base::boat_num++;
                                        Frame::money -= BOAT_PRICE;

                                        const auto [xx, yy] = entity_[purchase_id];
                                        printf("lboat %d %d\n", xx, yy);

                                        Boat b;
                                        b.trace_x = b.x = xx;
                                        b.trace_y = b.y = yy;
                                        b.trace_dir = b.dir = RIGHT;
                                        b.goods_num = 0;
                                        b.id = Frame::boats.size();

                                        Frame::boats.push_back(b);
                              }

                              return true;
                    }

                    template <typename... TArgs>
                    inline void emplace_back(TArgs&... __args)   {
                              entity_.emplace_back(std::forward<TArgs>(__args)...);
                              good_in_range.push_back({0, 0});
                    }
                    inline void append_good(const int id, const Good &good) const   {
                              (good_in_range[id].num)++;
                              (good_in_range[id].price) += good.price;
                    }
                    void remove_good(const int id, const Good &good) const   {
                              (good_in_range[id].num)--;
                              (good_in_range[id].price) -= good.price;
                    }
                    inline double find_purchase() const   {
                              double density_ = 0;
                              int ret = 0, ptr = 0;
                              for (auto &[num, price] : good_in_range) {
                                        double den_ = (double)price / (double)num;
                                        if (den_ > density_) {
                                                  density_ = den_;
                                                  ret = ptr;
                                        }
                                        ptr++;
                              }
                              return ret;
                    }

                    decltype(auto) operator[] (int index) { return entity_[index]; }
                    decltype(auto) begin() { return entity_.begin(); }
                    decltype(auto) end() { return entity_.end(); }
                    decltype(auto) rbegin() { return entity_.rbegin(); }
                    decltype(auto) rend() { return entity_.rend(); }

                    decltype(auto) size() { return entity_.size(); }
          private:

                    std::vector< std::tuple<int, int> > entity_;
                    struct Record { int num, price; };
                    mutable std::vector<Record> good_in_range;
          };


          void Base::ProcessMap()   {
                    for (int i = 0; i < N; ++i) {
                              for (int j = 0; j < N; ++j) {
                                        if (grid[i][j] == 'R') {
                                                  robot_purchase_point.emplace_back(i, j);
                                        }
                                        else if (grid[i][j] == 'S') {
                                                  boat_purchase_point.emplace_back(i, j);
                                        }
                              }
                    }
          }

          /**
           * Base部分
          */

          BerthStor Base::berths = BerthStor();
          int Base::boat_capacity = 0;
          // char **Base::grid = NULL;
          std::vector<std::string> Base::grid = std::vector<std::string>();

          int Base::robot_num = 0;
          int Base::boat_num = 0;
          int Base::berth_num = 0;
          int Base::goods_num = 0;

          PurchasePoint<Robot> Base::robot_purchase_point = PurchasePoint<Robot>();
          PurchasePoint<Boat> Base::boat_purchase_point = PurchasePoint<Boat>();

          /**
           * Frame部分
          */

          int Frame::id = 0;
          int Frame::gap_id = 0;
          int Frame::money = 0;
          GoodStor Frame::goods = GoodStor();
          std::vector<Robot> Frame::robots = std::vector<Robot>();
          std::vector<Boat> Frame::boats = std::vector<Boat>();

          std::vector<int> buffer;
          int Frame::update()   {
                    int crt_id, good_num;
                    if (scanf("%d%d", &crt_id, &money) == EOF) return -1;
                    gap_id = crt_id - id;

                    scanf("%d", &good_num);
                    buffer.clear();
                    for (auto &&[k, v] : goods) {
                              v.live -= gap_id;
                              if (v.live <= 0) {
                                        buffer.push_back(k);
                                        Base::robot_purchase_point.remove_good(v.purchase_rank, v);
                              }
                    }
                    for (auto &x : buffer) goods.erase(x);
                    id = crt_id;

                    for (int i = 0; i < good_num; ++i) {
                              int x, y, price;
                              scanf("%d%d%d", &x, &y, &price);

                              // record good when price bigger than 0
                              if (price != 0) goods[as_key(x, y)] = ({
                                        int distance_ = INT_MAX, pur_id = -1;
                                        for (int i = 0; i < Base::robot_purchase_point.size(); ++i) {
                                                  const auto [xx, yy] = (Base::robot_purchase_point)[i];
                                                  int dis_ = std::abs(xx - x) + std::abs(yy - y);
                                                  if (dis_ < distance_) {
                                                            distance_ = dis_;
                                                            pur_id = i;
                                                  }
                                        }

                                        // good属于离他最近的purchase
                                        auto ret = Good(x, y, price, pur_id, 1000);
                                        Base::robot_purchase_point.append_good(pur_id, ret);
                                        
                                        // 返回good
                                        std::move(ret);
                              }); 
                              else {
                                        if (goods.find_(x, y)) goods.erase(as_key(x, y));
                              }
                    }

                    int robot_num;
                    scanf("%d", &robot_num);
                    Base::robot_num = robot_num;
                    robots.resize(robot_num);     // 确保有足够的位置
                    for (int i = 0; i < robots.size(); ++i) {
                              int id;
                              scanf("%d", &id);

                              robots[id].id = id;
                              scanf("%d%d%d", &robots[id].goods_num, &robots[id].x, &robots[id].y);
                    }

                    int boat_num;
                    scanf("%d", &boat_num);
                    boats.resize(boat_num);       // 确保有足够的位置
                    for (int i = 0; i < boats.size(); ++i) {
                              scanf("%d%d%d%d%d%d", &boats[i].id, &boats[i].goods_num, &boats[i].x, &boats[i].y, &boats[i].dir, &boats[i].status);
                    }

                    return id;
          }
}

/**
 * 扩充的Base以及Frame
 * 把原来的基本数据和扩充的新数据隔离开，除非必须在原来的Base和Frame上进行更改，或者这样扩展影响效率
*/

struct MyBase: public BaseElem::Base {
          static std::vector< std::vector<int> > zlocks_;

          static inline void init()   {
                    Base::init();
                    zlocks_.resize(N,std::vector<int>(N, 0));
          }
};
std::vector< std::vector<int> > MyBase::zlocks_ = std::vector< std::vector<int> >();

struct MyFrame: public BaseElem::Frame {
          static int clock_for_margin_func;
          static int clock_id, price_count;
          static int need_robot, need_boat;
          static double margin_growth;
          static int router_times;

          static int update()  ;
          static inline void init()   { 
                    Frame::init(); 

                    MyFrame::clock_id = MyFrame::id;
          }

          // 购买的动作，是否购买机器人或者船......
          static inline void purchase_action()  ;
};
int MyFrame::clock_for_margin_func { 1 };
int MyFrame::clock_id { 0 };
int MyFrame::price_count { 0 };
int MyFrame::need_robot { INIT_ROBOT_NUM };
int MyFrame::need_boat { INIT_BOAT_NUM };
int MyFrame::router_times { 0 };

double MyFrame::margin_growth { 0 };

/**
 * 主循环
*/

struct Loop {
          void init() { 
                    MyBase::init();
                    MyFrame::init(); 
          }
          inline int Run() {
                    if (MyFrame::update() == -1) return -1;
                    puts("OK"); fflush(stdout);
                    return 0;
          }
          void free() { 
                    MyBase::dealloc(); 
                    MyFrame::dealloc();
          }

          Loop() { init(); }
          ~Loop() { display(LOOP EXECUTE OVER!\n); free(); }
};

int main() {
          try {
                    for (Loop self;;)
                    if (self.Run() == -1) return 0;
          } catch (std::exception e) {
                    display(ABORTED ERROR: %s\n, e.what());
          }
}


int MyFrame::update()   {
          if (Frame::update() == -1) return -1;

          display(DEBUG::: Frame update.\n);
          for (auto &[x, y] : MyBase::robot_purchase_point) {
                    display(Robot purchase {%d %d}.\n, x, y);
          }

          display(Boat purchase point size %ld.\n, MyBase::boat_purchase_point.size());
          for (auto &[x, y] : MyBase::boat_purchase_point) {
                    display(Boat purchase {%d %d}.\n, x, y);
          }

          // 扩充的update信息
          // ...

          // 准备动作/ 机器人动作/ 船最后动
          perfix_action();
          
          std::vector<std::thread> parallel_tasks(2);
          parallel_tasks[0] = std::thread(robot_action);
          parallel_tasks[1] = std::thread(boat_action);
          for (int i = 0; i < 2; ++i) parallel_tasks[i].join();

          // robot_action();

          // boat_action();
                    
          // 购买动作
          purchase_action();
          suffix_action();

          // 提交这一帧的操作
          CHECK_OK();
          return Frame::id;
}

/**
 * 当能够购买的数量大于0，并且当前存在数量小于最大数量
 * 购买机器人或者船
*/
void MyFrame::purchase_action()   {
          if (need_boat > 0 && MyBase::boat_num < BOAT_LIMIT) {
                    int id = std::rand() % MyBase::boat_purchase_point.size();

                    if (MyBase::boat_purchase_point.purchase(id)) {
                              need_boat--;
                    } else {
                              // 强制先买船
                              return;
                    }
          }

          if (need_robot > 0 && MyBase::robot_num < ROBOT_LIMIT) {
                    int id = MyBase::robot_purchase_point.find_purchase();

                    if (MyBase::robot_purchase_point.purchase(id)) {
                              need_robot--;
                    }
          }
}

/**
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * == == == == == == == == == == == == == == == == == == == == == == == 算 法 的 具 体 实 现 == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
*/

// 上是x-1，下是x+1，右是y+1，左是y-1
Path __trace_back(std::vector<std::vector<int>> &map, int x, int y)   {
          Path ret;
          ret.tar_x = x; ret.tar_y = y;
          while (map[x][y] != POINT) {
                    ret.emplace_back(map[x][y]);
                    int dir = map[x][y];
                    switch (dir) {
                              case UP: x++; break;
                              case DOWN: x--; break;
                              case LEFT: y++; break;
                              case RIGHT: y--; break;
                              default: {
                                        fprintf(stderr, "![important] ::: Unknown direction!\n");
                              }
                    }

                    if (x < 0 || x >= N || y < 0 || y >= N) {
                              display(FATLE ERROR::: out of boundary!\n);
                              
                    }
          }
          // ret.distance = distance;
          ret.cursor = ret.size() - 1;
          return ret;
}

/**
 * 筛选路径，只保留一条路径并且对路径进行避免碰撞的操作
 *        ::: 如果不在规划路径时进行碰撞规避，那就只筛选出一条路径
*/
inline Path __sift_and_lock(std::vector<Path> &path)   {
          if (path.empty()) return Path();        // 没有找到路径
          if (path.size() == 1) return path[0];     // 目前router_dij就找了一条路

          // 比较多条路径
          double score_ = (double)path[0].tar_price / (double)path[0].get_distance();
          int ret_ = 0;
          for (int i = 1; i < path.size(); ++i) {
                    auto &p = path[i];
                    double score = (double)p.tar_price / (double)p.get_distance();
                    if (score > score_) {
                              ret_ = i;
                              score_ = score;
                    }
          }
          return path[ret_];
}

typedef bool (*check_position) (int, int);
inline bool __check_berth(int x, int y)   { return (MyBase::grid[x][y] == 'B'); }
inline bool __check_good(int x, int y)   { return MyFrame::goods.find_(x, y); }

typedef bool (*check_can_move) (char &);
inline bool __both_robot_boat_move(char &c)   { return c == 'C' || c == 'c'; }
inline bool __robot_can_move(char &c)   { return c == '.' || c == '>' || c == 'R' || c == 'B' || __both_robot_boat_move(c); }

std::set<int> seen;


template<>
struct std::hash<Position> {
          std::size_t operator() (Position const &p) const noexcept {
                    return std::hash<int>{}(std::get<0>(p)) ^ std::hash<int>{}(std::get<1>(p));
          }
};

// 寻找路径，默认只找最近的
const Path router_dij(Robot &robot, size_t cap = 1)   {
          auto &map_ = MyBase::grid;
          // 加锁的操作转移到__sift_and_lock()中，所以这里设置不可修改locks_
          // const auto &locks_ = MyBase::locks_;

          check_position __check = robot.goods_num ? (__check_berth) : (__check_good);
          check_can_move __can_move = __robot_can_move;

          std::vector<std::vector<int>> trace_(N, std::vector<int>(N, -1));
          std::vector<std::vector<int>> distance_(N, std::vector<int>(N, INT_MAX));

          trace_[robot.x][robot.y] = POINT;
          distance_[robot.x][robot.y] = 0;

          seen.clear();
          auto ret = std::vector<Path>();

          auto wait_ = std::unordered_set<Position>(); wait_.emplace(robot.x, robot.y);

          CYCLE__: while (wait_.size()) {
                    std::unordered_set<Position> buff_wait;

                    for (auto [x, y] : wait_) {
                              // auto [x, y] = pos;

                              if (__check(x, y) && seen.find(as_key(x, y)) == seen.end()) {
                                        // 可以容忍的距离/ 去泊点无所谓距离/ 去货物只能容忍货物的存在帧
                                        int tolance_ =  robot.goods_num ? -1 : MyFrame::goods[as_key(x, y)].live;
                                        if (tolance_ == -1 || distance_[x][y] <= tolance_) {
                                                  // display(Find target_......\n);
                                                  auto path = __trace_back(trace_ , x, y);

                                                  path.type_ = robot.goods_num ? BERTH : GOOD;
                                                  path.tar_price = MyFrame::goods[as_key(x, y)].price;

                                                  ret.emplace_back(path);
                                                  seen.emplace(as_key(x, y));

                                                  if (ret.size() >= cap) goto RET__;
                                        } else {
                                                  goto RET__;
                                        }
                              }

                              int distance = distance_[x][y] + 1;

                              if (x - 1 >= 0 && __can_move(map_[x-1][y]) && distance < distance_[x-1][y]) {
                                        trace_[x-1][y] = UP;
                                        distance_[x-1][y] = distance;
                                        buff_wait.emplace(x-1, y);
                              }
                              if (x + 1 < N && __can_move(map_[x+1][y]) && distance < distance_[x+1][y]) {
                                        trace_[x+1][y] = DOWN;
                                        distance_[x+1][y] = distance;
                                        buff_wait.emplace(x+1, y);
                              }
                              if (y - 1 >= 0 && __can_move(map_[x][y-1]) && distance < distance_[x][y-1]) {
                                        trace_[x][y-1] = LEFT;
                                        distance_[x][y-1] = distance;
                                        buff_wait.emplace(x, y-1);
                              }
                              if (y + 1 < N && __can_move(map_[x][y+1]) && distance < distance_[x][y+1]) {
                                        trace_[x][y+1] = RIGHT;
                                        distance_[x][y+1] = distance;
                                        buff_wait.emplace(x, y+1);
                              }
                    }

                    wait_.clear();
                    wait_ = std::move(buff_wait);
          }

          RET__:
          return __sift_and_lock(ret);
}


/**
 * 船在一个状态转换的其他状态
*/
const std::vector<std::tuple<int, int, int>> to_right_ { {0, 2, DOWN}, {0, 1, RIGHT}, {1, 1, UP} };
const std::vector<std::tuple<int, int, int>> to_left_ { {0, -1, LEFT}, {0, -2, UP}, {-1, -1, DOWN} };
const std::vector<std::tuple<int, int, int>> to_up_ { {-1, 0, UP}, {-1, 1, LEFT}, {-2, 0, RIGHT} };
const std::vector<std::tuple<int, int, int>> to_down_ { {1, 0, DOWN}, {2, 0, LEFT}, {1, -1, RIGHT} };

// 船的trace_back/ TODO: 
/**
 * 当前cdir_和上一个的方向dir_进行比较
 * 1. 右转 右->下/ 下->左/ 左->上/ 上->右
 * 2. 直行 相等
 * 3. 左转 右->上/ 下->右/ 左->下/ 上->左
*/
Path __trace_back_boat(std::vector< std::vector< std::vector<int> > > &direction_, int x, int y, int dir)   {
          auto ret = Path();
          ret.tar_x = x, ret.tar_y = y; // 目标位置
          // 确定船的Path中每一个方向
          auto confirm_dir_ = [&](int last_dir, int crt_dir) {
                    if (last_dir == crt_dir) return SHIP;
                    switch (last_dir) {
                              case RIGHT: return crt_dir == DOWN ? CLOCK : ANTI;
                              case DOWN: return crt_dir == LEFT ? CLOCK : ANTI;
                              case LEFT: return crt_dir == UP ? CLOCK : ANTI;
                              case UP: return crt_dir == RIGHT ? CLOCK : ANTI;
                              default: {
                                        return SHIP;
                              }
                    }
          };
          // 进入上一个追踪点
          auto back_point_ = [](int &x, int &y, int &cdir_, int dir_) {
                    auto &to_next_ =    dir_ == RIGHT       ? to_right_ : (
                                        dir_ == LEFT        ? to_left_ : (
                                        dir_ == UP          ? to_up_ : (
                                                            to_down_ // dir == DOWN
                                        )
                              )
                    );
                    for (auto [dx, dy, ddir_] : to_next_) {
                              if (ddir_ != cdir_) continue;
                              x -= dx, y -= dy;
                              cdir_ = dir_;
                              return;
                    }
          };

          // 不是POINT就代表存在上一个状态
          while (direction_[x][y][dir] != POINT) {
                    ret.emplace_back(confirm_dir_(direction_[x][y][dir], dir));
                    back_point_(x, y, dir, direction_[x][y][dir]);
          }

          ret.cursor = ret.size() - 1;
          return ret;
}

// args 可以为任何类型参数/ 自定义函数可能需要不同类型的参数
typedef bool (*check_position_boat) (int x, int y, int dir, void *);
// 默认方法 
inline bool __check_position_boat_default(int x, int y, int dir, void *args) {
          // 最简单的就是判断 核心点 到达K点
          return MyBase::grid[x][y] == 'K';
}
inline bool __check_position_boat_T(int x, int y, int dir, void *args) {
          return MyBase::grid[x][y] == 'T';
}
// 传入args为一个Berth的指针/ 找到去指定Berth的路
inline bool __check_position_boat_1 (int x, int y, int dir, void *args) {
          Berth *berth = (Berth *)args;
          if (MyBase::grid[x][y] == 'K') {
                    if (std::abs(x - berth->x) < 9 && std::abs(y - berth->y) < 9) return true;
          }
          return false;
}

/**
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * 船的碰撞避免，时间窗方法
*/

// 时间窗口
typedef std::array<int, 2> __WINDOW;
class WINDOW {
public:
          WINDOW(): win_{-1, -1} {}
          // 这个会返回存储内容的复制
          inline int get_start() { return win_[0]; }
          inline int get_end() { return win_[1]; }
          // as_ref方便对内容进行修改
          inline decltype(auto) get_start_as_ref() { return (win_[0]); };
          inline decltype(auto) get_end_as_ref() { return (win_[1]); }
private:
          __WINDOW win_;
};
// vector会在栈内保存基础类型，包括指针，对于开辟的动态内存，调用malloc calloc mmap等方法在堆内申请内存
// N x N x MAX_BOAT_LIMIT x 2 的锁，BOAT_LIMIT不应该超过MAX_BOAT_LIMIT
std::vector< std::vector< std::vector<WINDOW> > > blocks_ = 
          std::vector< std::vector< std::vector<WINDOW> > >(N, std::vector< std::vector<WINDOW> >(N, std::vector<WINDOW>(MAX_BOAT_LIMIT)));

// 输入trace_x和trace_y，根据当前动作与朝向判断下一帧的位置与朝向
void __next_position_boat(int &x, int &y, int &dir, int action) {
          switch (action) {
                    case SHIP: ({
                              switch (dir) {
                                        case UP: x -= 1; break;
                                        case DOWN: x += 1; break;
                                        case RIGHT: y += 1; break;
                                        case LEFT: y -= 1; break;
                              }
                    }); break;
                    case CLOCK: ({
                              switch (dir) {
                                        case UP: x -= 2; dir = RIGHT; break;
                                        case DOWN: x += 2; dir = LEFT; break;
                                        case RIGHT: y += 2; dir = DOWN; break;
                                        case LEFT: y -= 2; dir = UP; break;
                              }
                    }); break;

                    case ANTI: ({
                              switch (dir) {
                                        case UP: dir = LEFT; x -= 1; y += 1; break;
                                        case DOWN: dir = RIGHT; x += 1; y -= 1; break;
                                        case RIGHT: dir = UP; x += 1; y += 1; break;
                                        case LEFT: dir = DOWN; x -= 1; y -= 1; break;
                              }
                    }); break;

                    default: {
                              // 这种错误直接导致后面全部乱套
                              display(![important] ::: Unknown action for boat [%d]\n, action);
                              return;
                    }
          }
}

// 为船 boat 的路径 path 上锁
void __trace_lock (Boat &boat, Path &path) {
          
          // x y点加锁，时间为t
          auto lock_point_ = [&](int x, int y, int t) {
                    if (blocks_[x][y][boat.id].get_start() == -1) {
                              // 第一次进入记录到达时间
                              blocks_[x][y][boat.id].get_start_as_ref() = t;
                    }
                    // 刷新离开时间
                    blocks_[x][y][boat.id].get_end_as_ref() = t;
          };

          // 对船在 x y 方向为 dir 时上锁 t
          auto lock_position_ = [&](int x, int y, int dir, int t) {
                    switch(dir) {
                              case UP:
                              //  [x y] [x-1 y] [x-2 y] [x y+1] [x-1 y+1] [x-2 y+1]
                              lock_point_(x, y, t), lock_point_(x-1, y, t), lock_point_(x-2, y, t);
                              lock_point_(x, y+1, t), lock_point_(x-1, y+1, t), lock_point_(x-2, y+1, t);
                              break;

                              case DOWN:
                              // [x y] [x+1 y] [x+2 y] [x y-1] [x+1 y-1] [x+2 y-1]
                              lock_point_(x, y, t), lock_point_(x+1, y, t), lock_point_(x+2, y, t);
                              lock_point_(x, y-1, t), lock_point_(x+1, y-1, t), lock_point_(x+2, y-1, t);
                              break;

                              case LEFT:
                              // [x y] [x y-1] [x y-2] [x-1 y] [x-1 y-1] [x-1 y-2]
                              lock_point_(x, y, t), lock_point_(x, y-1, t), lock_point_(x, y-2, t);
                              lock_point_(x-1, y, t), lock_point_(x-1, y-1, t), lock_point_(x-1, y-2, t);
                              break;

                              case RIGHT:
                              // [x y] [x y+1] [x y+2] [x+1 y] [x+1 y+1] [x+1 y+2]
                              lock_point_(x, y, t), lock_point_(x, y+1, t), lock_point_(x, y+2, t);
                              lock_point_(x+1, y, t), lock_point_(x+1, y+1, t), lock_point_(x+1, y+2, t);
                              break;

                              default: {
                                        display(![important] Unknown direction (lock_position_())\n);
                              }
                    }
          };
          
          // 从这里开始加锁，刚开始脚下是0
          int dir_ = boat.dir;
          int x_ = boat.x, y_ = boat.y;
          int time_ = 0;
          // reverse iterator，反向遍历路径
          for (auto iter = path.rbegin(); iter != path.rend(); ++iter, ++time_) {
                    lock_position_(x_, y_, dir_, time_);        // 当前位置加锁

                    // 去到下一个位置
                    __next_position_boat(x_, y_, dir_, *iter);
          }
}

// 在碰撞后dept时清空锁
void __force_unlock_dept(Boat &boat) {

}

// 船当前状态，在move前调用
void __forward_unlock(Boat &boat) {
          auto unlock_ = [&](int x, int y) {
                    blocks_[x][y][boat.id].get_start_as_ref() = -1;
                    blocks_[x][y][boat.id].get_end_as_ref() = -1;
          };

          switch (boat.dir) {
                    case UP: unlock_(boat.x, boat.y + 1); break;
                    case RIGHT: unlock_(boat.x + 1, boat.y); break;
                    case DOWN: unlock_(boat.x, boat.y - 1); break;
                    case LEFT: unlock_(boat.x - 1, boat.y); break;
                    default: {
                              display(![important] Unknown direction (__forward_unlock(boat %d))\n, boat.id);
                              return;
                    }
          }
          unlock_(boat.x, boat.y);
}







/**
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
*/


// 船寻路
const Path 
router_boat(
          Boat &boat, 
          const check_position_boat __check_position, // = __check_position_boat_default, 
          void *args // = NULL   // 比如传入一个Berth，判断是否到达这个berth
                              // 或者传入一个id，判断是否到达指定id的berth
)   {
          auto &map_ = MyBase::grid;
          // 引入锁/ ! zlocks_是单层的
          auto &locks_ = MyBase::zlocks_;

          /**
           * 初始化
          */
          std::vector< std::vector< std::vector<int> > > direction_(N, std::vector(N, std::vector(4, -1)));
          std::vector< std::vector< std::vector<int> > > distance_(N, std::vector(N, std::vector(4, INT_MAX)));

          direction_[boat.x][boat.y][boat.dir] = POINT;
          distance_[boat.x][boat.y][boat.dir] = 0;

          // 检查船在 x y 处方向 dir 能否存在/ 判读锁
          auto __check_can_move = [&](int x, int y, int dir) {
                    auto __pos_valid = [&](int x, int y) {
                              // 检查越界
                              if (!(x >= 0 && x < N && y >= 0 && y < N)) return false;
                              return map_[x][y] == '*' || map_[x][y] == '~' || map_[x][y] == 'K' || map_[x][y] == 'C' || map_[x][y] == 'c' || map_[x][y] == 'T';
                    };
                    switch (dir) {
                              case UP: return __pos_valid(x - 2, y) && __pos_valid(x - 2, y + 1);
                              case DOWN: return __pos_valid(x + 2, y) && __pos_valid(x + 2, y - 1);
                              case LEFT: return __pos_valid(x, y - 2) && __pos_valid(x - 1, y - 2);
                              case RIGHT: return __pos_valid(x, y + 2) && __pos_valid(x + 1, y + 2);
                    }
                    return false;
          };
          // 检查这个位置是否碰到了主航道 主航道 dis 2，普通航道 dis 1
          auto __check_distance_comsume = [&](int x, int y, int dir) {
                    int ret = INT_MAX >> 1;
                    switch (dir) {
                              case UP: ret = (map_[x - 2][y] == 'K' || map_[x - 2][y] == '~' || map_[x - 2][y] == 'c' || map_[x - 2][y + 1] == '~' || map_[x - 2][y + 1] == 'c') ? 2
                                        : 1;
                                        break;
                              case DOWN: ret = (map_[x + 2][y] == 'K' || map_[x + 2][y] == '~' || map_[x + 2][y] == 'c' || map_[x + 2][y - 1] == '~' || map_[x + 2][y - 1] == 'c') ? 2
                                        : 1;
                                        break;
                              case LEFT: ret = (map_[x][y - 2] == 'K' || map_[x][y - 2] == '~' || map_[x][y - 2] == 'c' || map_[x - 1][y  - 2] == '~' || map_[x - 1][y - 2] == 'c') ? 2 
                                        : 1;
                                        break;
                              case RIGHT: ret = (map_[x][y + 2] == 'K' || map_[x][y + 2] == '~' || map_[x][y + 2] == 'c' || map_[x + 1][y + 2] == '~' || map_[x + 1][y + 2] == 'c') ? 2 
                                        : 1;
                                        break;
                    }
                    return ret;
          };

          // position, direction
          std::set<std::tuple<int, int, int>> wait_;
          wait_.emplace(boat.x, boat.y, boat.dir);

          /**
           * 寻路的主体部分
          */
          while (wait_.size()) {
                    auto buff_wait = std::set<std::tuple<int, int, int>>();
                    for (auto &[x, y, dir] : wait_) {
                              // 首先检查这个位置是不是到了目标
                              if (__check_position(x, y, dir, args)) {
                                        // 构造并返回找到的路径
                                        return __trace_back_boat(direction_, x, y, dir);
                              }

                              // 再对这个位置的下一层做遍历
                              auto &to_next_ =    dir == (int)RIGHT   ? to_right_ : (
                                                  dir == (int)LEFT    ? to_left_ : (
                                                  dir == (int)UP      ? to_up_ : (
                                                                      to_down_ // dir == DOWN
                                                  )
                                        )
                              );

                              // 对于三种转换/ dx dy是变化量，ddir_是变化后的朝向
                              for (auto &[dx, dy, ddir_] : to_next_) {
                                        int xx = x + dx, yy = y + dy; // 变化后的 核心点 位置
                                        if (!__check_can_move(xx, yy, ddir_)) continue;

                                        int dis_ = distance_[x][y][dir] + __check_distance_comsume(xx, yy, ddir_);
                                        if (dis_ < distance_[xx][yy][ddir_]) {
                                                  distance_[xx][yy][ddir_] = dis_;
                                                  direction_[xx][yy][ddir_] = dir;        // 记录来源状态，可能由三种状态转换而来
                                                                      // 方便trace出Path

                                                  buff_wait.emplace(xx, yy, ddir_);
                                        }
                              }
                    }
                    wait_ = buff_wait;
          }
          // DEBUG
          return Path();
}

// 在机器人和船行动前先进行的操作
void perfix_action()   {
          MyFrame::router_times = 0;    // 找路次数清零
          // 检查机器人位置的正确性
          for (auto &robot : MyFrame::robots) {
                    if (robot.x != robot.trace_x || robot.y != robot.trace_y) {
                              // 归位
                              robot.collision = true;
                              robot.trace_x = robot.x;
                              robot.trace_y = robot.y;
                    }
          }

          for (auto &boat : MyFrame::boats) {
                    if (boat.collition) {
                              boat.trace_x = boat.x;
                              boat.trace_y = boat.y;
                              boat.trace_dir = boat.dir;
                              boat.collition = false;       // 恢复状态，下面调用boat_action就可以行动了

                    } else if (boat.x != boat.trace_x || boat.y != boat.trace_y || boat.dir != boat.trace_dir) {
                              display(COLLI::: boat collition is true[%d]\n, boat.id);
                              boat.collition = true;        // 交给boat_action，boat_action后下一帧回到这里
                                        // 进行上面if内的动作
                    } // collition为false并且trace正确，没有发生碰撞
          }
}
// std::vector<Robot *> after_move;
// 机器人的操作/ 为每个机器人寻路/ 每个机器人移动或拿起放下货物/ [TODO: 需要修改为多线程并发操作 X]
std::vector<std::thread> tasks_;
void robot_action()   {
          for (auto &robot : MyFrame::robots) {

                    if (robot.collision) robot.current_path.clear(), robot.stack_.clear(), robot.collision = false;  // 碰撞之后重新寻路

                    // 机器人寻路，一帧中最多找ROUTER_LIMIT_PER_FRAME次路
                    if (robot.current_path.empty() && MyFrame::router_times < ROUTER_LIMIT_PER_FRAME) {
                              auto router = robot.goods_num ? router_dij(robot) : router_dij(robot, CMP_TARGET_NUM);

                              MyFrame::router_times++;      // 找路次数加一
                              if (router.empty()) {
                                        // 冗余 拿/放 操作
                                        if (robot.goods_num == 0) {
                                                  robot.get();
                                                  MyFrame::goods.erase(as_key(robot.x, robot.y));
                                        } else robot.pull();
                                        continue;
                              
                              }

                              // 目前就一条路径
                              robot.current_path = std::move(router);
                              if (robot.current_path.type_ == GOOD) {
                                        int key = as_key(robot.current_path.tar_x, robot.current_path.tar_y);
                                        MyFrame::goods.erase(key);
                              } 
                    }


                    if (robot.current_path.empty()) continue;
                    // 机器人移动
                    
                    auto &path = robot.current_path;

                    // 当存在临时路径，先走临时路径
                    if (robot.stack_.size()) {
                              auto dir_ = robot.stack_.back();
                              robot.stack_.pop_back();
                              
                              robot.move(dir_);
                    } else robot.move(path.get_direction());
                    
                    if (path.done() && robot.stack_.empty()) { //路径走完了
                              if (robot.goods_num) robot.pull();
                              else robot.get();
                              robot.current_path.clear();   // 走完整条路径，忘掉之前的路径
                    }
          }
}

Berth *__alloc_berth_for_boat(Boat &boat)   {
          int score = -1;
          Berth *ret = nullptr;
          for (auto &berth : MyBase::berths) {
                    if (berth.free_ == false) continue;
                    int score_ = berth.crt_num;
                    display(SCORE::: berth %d has crt_num %d\n, berth.id, berth.crt_num);
                    if (score_ > score) score = score_, ret = &berth;
          }
          return ret;
}

void display_path(const Path &path) {
          for (auto x : path) {
                    switch (x) {
                              case CLOCK: display(CLOCL|); break;
                              case ANTI: display(ANTI|); break;
                              case SHIP: display(SHIP|); break;
                              case LEFT: display(LEFT|); break;
                              case RIGHT: display(RIGHT|); break;
                              case UP: display(UP|); break;
                              case DOWN: display(DOWN|); break;
                              default: {
                                        display(UNKNOWN|);
                              }
                    }
          }
          display(\n);
}
// 船的操作/ 分配泊点/ 装卸货物
void boat_action()   {
          for (auto &boat : MyFrame::boats) {
                    if (boat.collition) {
                              boat.dept();                  // 闪现到最近的主航道
                              boat.current_path.clear();    // 忘记旧路径
                              __force_unlock_dept(boat);    // 清空旧路径的锁
                              continue;
                    }

                    if (boat.status == 0) { // 正常状态 
                              // 路径为空，代表 空闲
                              if (boat.current_path.empty()) {
                                        auto tar_berth = __alloc_berth_for_boat(boat);
                                        if (tar_berth == nullptr) { continue; }

                                        if (!boat.can_leave) {
                                                  display(ROUTER:: To berth %d [num %d]...\n, tar_berth->id, tar_berth->crt_num);
                                                  boat.current_path = router_boat(boat, __check_position_boat_1, tar_berth);  // 寻找一个最近的泊点
                                                  display(ROUTER:: boat router tarx %d tary %d\n, boat.current_path.tar_x, boat.current_path.tar_y);

                                                  auto ret = MyBase::berths.end();
                                                  // 找到占用的泊位，然后占用泊位
                                                  if (boat.current_path.empty()) {        // 当前位置？寻路可能有些问题？
                                                            display(::: Empty Boat Path\n);
                                                            ret = MyBase::berths.iter_find(boat.x, boat.y, [](int x, int y, Berth &berth) {
                                                                      return std::abs(x - berth.x) < 9 && std::abs(y - berth.y) < 9;
                                                            });
                                                  } else { // 设置占用泊位
                                                            ret = MyBase::berths.iter_find(boat.current_path.tar_x, boat.current_path.tar_y, [](int x, int y, Berth &berth) {
                                                                      return std::abs(x - berth.x) < 9 && std::abs(y - berth.y) < 9;
                                                            });
                                                  }

                                                  // 找到一个泊点
                                                  if (ret != MyBase::berths.end()) {
                                                            boat.set_aim(&(*ret)); // 设置状态/ 在装载状态结束时恢复这些状态位
                                                  }
                                        } else boat.current_path = router_boat(boat, __check_position_boat_T, NULL);
                              }
                              boat.move();

                    } else if (boat.status == 1) {          // 恢复状态
                              
                    } else {  // 装载状态
                              if  (boat.aim->crt_num <= boat.goods_num || boat.goods_num >= MyBase::boat_capacity) {
                                        boat.can_leave = true;
                                        boat.aim->crt_num = std::max(boat.aim->crt_num - boat.goods_num, 0);
                              }
                              
                              if (boat.can_leave) {
                                        boat.dept();        // 看看能去哪
                                        boat.release_aim();
                                        boat.current_path.clear();
                              }
                    }
          }
}

void suffix_action()   {
          
}


/**
 * 扩展的Robot动作
*/

long total_price_ = 0;
void Robot::get()   {
          Robot__::get();
          
          current_price = current_path.size() ? current_path.tar_price : (
                    MyFrame::goods.find_(x, y) ? 
                              ({
                                        auto &good = MyFrame::goods[as_key(x, y)];
                                        MyBase::robot_purchase_point.remove_good(good.purchase_rank, good);
                                        good.price;
                              }) : 0
          );

          total_price_ += current_price;
}

void Robot::pull() const   {
          Robot__::pull();

          // 更新MyFrame判断是否增加机器人的数据
          MyFrame::price_count += current_price;
          // 当记时器结束的时候根据 上一次增加机器人后 每帧平均价值判断是否应该增加机器人/ 现在的逻辑是只要大于没增加的就再次增加，
                    // 可以设置一个阈值，当增加了多少才选择再次增加机器人
          if((MyFrame::clock_for_margin_func -= 1) == 0) {
                    double current_growth_ = (double)MyFrame::price_count / (double)(MyFrame::id - MyFrame::clock_id);
                    if (current_growth_ <= MyFrame::margin_growth) 
                              MyFrame::clock_for_margin_func = MyBase::robot_num;
                    else {
                              MyFrame::need_robot++;
                              if (((MyFrame::need_boat + MyFrame::robots.size()) >> 1) > (MyFrame::need_boat + MyFrame::boats.size())) 
                                        MyFrame::need_boat++;         // 船的数量为机器人的一半/ 为2的时候是1/4/ 为3的时候是1/8
                              MyFrame::clock_for_margin_func = MyBase::robot_num + 1;     // 刷新间隔增加
                    }

                    MyFrame::clock_id = MyFrame::id;
                    MyFrame::price_count = 0;
          }

          // 记录放货
          int tar_x, tar_y;
          if (current_path.size()) tar_x = current_path.tar_x, tar_y = current_path.tar_y;
          else tar_x = x, tar_y = y;

          auto ret = MyBase::berths.iter_find(tar_x, tar_y, [](int x, int y, Berth &berth) {
                    return std::abs(x - berth.x) < 9 && std::abs(y - berth.y) < 9;
          });

          if (ret != MyBase::berths.end()) {
                    display(PUT::: berth is num + 1 [%d]\n, (*ret).id);
                    (*ret).crt_num += 1;
          }
}

inline void __force_lock(int x, int y)   {
          if (MyBase::grid[x][y] == 'c') {
                    auto &map_ = MyBase::grid;
                    bool tag = false;
                    if (!tag && x - 1 >= 0 && map_[x-1][y] == '.') tag = true;
                    if (!tag && x + 1 < N && map_[x+1][y] == '.') tag = true;
                    if (!tag && y - 1 >= 0 && map_[x][y-1] == '.') tag = true;
                    if (!tag && y + 1 < N && map_[x][y+1] == '.') tag = true;
                    if (tag) MyBase::zlocks_[x][y] = 1;
          } 
          else if (MyBase::grid[x][y] == '>' || MyBase::grid[x][y] == 'R')  {} 
          else if (MyBase::grid[x][y] == '~' || MyBase::grid[x][y] == 'S') {} 
          else {    // 会发生碰撞的地界
                    MyBase::zlocks_[x][y] = 1;
          }
          // return true;
}
inline void __force_unlock(int x, int y)   { MyBase::zlocks_[x][y] = 0; }

// 用于选择优先避让方向
std::vector<int> direction_list {LEFT, RIGHT, UP, DOWN};
void Robot::move(int dir)   {
          // int otrace_x = trace_x, otrace_y = trace_y;
          auto &zlocks_ = MyBase::zlocks_;

          switch (dir) {
                    case UP: trace_x--; break;
                    case DOWN: trace_x++; break;
                    case LEFT: trace_y--; break;
                    case RIGHT: trace_y++; break;
                    default: {
                              display(::: Unknown direction[%d]......\n, dir);
                    }
          }

          if (zlocks_[trace_x][trace_y] == 0) {                                           // 正常完成移动
                    Robot__::move(dir);
                    // zlocks_[x][y] = 0, zlocks_[trace_x][trace_y] = 1;                     // 换锁
                    __force_lock(trace_x, trace_y), __force_unlock(x, y);
          } else {
                    // 前方有机器人挡路
                    stack_.emplace_back(dir);                                   // 记录没完成的路径

                    trace_x = x, trace_y = y;

                    int priority_ = (dir == UP || dir == DOWN) ? 0 : 2;                   // 首先垂直避让
                    for (int i = 0; i < 4; ++i, priority_ = (priority_ + 1) % 4) {
                              int ctrace_x = x, ctrace_y = y;                                       // 从当前方向开始
                              switch (direction_list[priority_]) {
                                        case UP: ctrace_x--; break;
                                        case DOWN: ctrace_x++; break;
                                        case LEFT: ctrace_y--; break;
                                        case RIGHT: ctrace_y++; break;
                              }

                              if (ctrace_x >= 0 && ctrace_x < N && ctrace_y >= 0 && ctrace_y < N &&
                                        MyBase::grid[ctrace_x][ctrace_y] == '#' || MyBase::grid[ctrace_x][ctrace_y] == '*' || MyBase::grid[ctrace_x][ctrace_y] == '~' ||
                                        MyBase::grid[ctrace_x][ctrace_y] == 'S' || MyBase::grid[ctrace_x][ctrace_y] == 'K' || MyBase::grid[ctrace_x][ctrace_y] == 'T') 
                              continue; // 机器人无法走海上和障碍物


                              // 发现周围的陆地并且没有机器人占用
                              if (zlocks_[ctrace_x][ctrace_y] == 0) {
                                        __force_lock(ctrace_x, ctrace_y), __force_unlock(x, y);
                                        trace_x = ctrace_x, trace_y = ctrace_y;                     // 更新trace
                                        Robot__::move(direction_list[priority_]);                   // 移动
                                        
                                        switch (direction_list[priority_]) {                        // 记录新增临时路径
                                                  case UP: stack_.emplace_back(DOWN); break;
                                                  case DOWN: stack_.emplace_back(UP); break;
                                                  case LEFT: stack_.emplace_back(RIGHT); break;
                                                  case RIGHT: stack_.emplace_back(LEFT); break;
                                        }
                                        break;                                                      // 完成
                              }
                    }
          }
}

void Boat::move()   {
          // Boat__::ship();
          if (current_path.empty()) return;

          auto &path = current_path;
          if (!path.done()) {
                    // 过去的路径解锁
                    __forward_unlock(*this);

                    int dir_ = path.get_direction();
                    // 追踪下一步应该处于的状态
                    __next_position_boat(trace_x, trace_y, trace_dir, dir_);

                    // 移动，下一帧更新的x y dir正常的话应该等于trace_[x y dir]
                    if (dir_ == SHIP) ship();
                    else rot(dir_);
          } else {
                    // 空船到泊点，靠泊
                    if (can_leave == false) berth();
                    else {
                              can_leave = false;
                              current_path.clear();
                    }
          }
}

