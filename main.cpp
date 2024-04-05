
/**
 * 需要解决的问题
 * 
 * 1. 制定泊点的路径/ 送货点的路径
 * 2. 并发
 * 3. 船路径加锁/ 
 * 4. 控制船和机器人的数量
 * 
 * 正在实现的东西
 * 
 * 
*/

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

/**
 * 线程池实现
*/

class ThreadPool {
public:
          struct Task {

          };
          // typedef std::priority_queue<>

          inline int append_tack() noexcept {

                    return task_id++;
          }

private:
          size_t task_id, min_id;
};



#define N 200
#define BOAT_PRICE 8000
#define ROBOT_PRICE 2000

#define ROBOT_LIMIT 10        // 设置一个最大量
#define BOAT_LIMIT 1         // 设置一个最大量

#define INIT_ROBOT_NUM 2      // 初始买几个机器人
#define INIT_BOAT_NUM 1       // 初始买几个船

#define ROUTER_LIMIT_PER_FRAME 5        // 每帧最多找几次路
#define CMP_TARGET_NUM        10         // 每次比较多少个货物确定要找的货物

#define display(msg, ...) ({ fprintf(stderr, #msg, ##__VA_ARGS__); })
#define CHECK_OK() ({ char okk[100]; scanf("%s", okk); })

typedef std::tuple<int, int> Position;
enum Direction {RIGHT = 0, LEFT, UP, DOWN, POINT};
constexpr int CLOCKWISE = 1 << 5;
enum Rotation {CLOCK = CLOCKWISE, ANTI, SHIP};

#define as_key(x, y) ((x << 10) | y)
#define as_position(key) ({ {key >> 10, key & 0x3FF} })

/**
 * 提前声明类型方便后续访问
*/

typedef std::vector<int> Path__;
class Path;

enum TraceType { GOOD, BERTH, None };

// 用来构造路径
Path __trace_back(int const map[N][N], int x, int y) noexcept;

/**
 * 路径指针，用来移动
 * tar参数为目标位置
 * TraceType为寻路的目标类型
 * distance为路径距离
*/
class Path: public Path__ {
public:
          template <typename... TArgs>
          Path(TArgs... args): Path__(std::forward<TArgs>(args)...), /*distance(0),*/ cursor(-1), tar_x(0), tar_y(0), type_(None) {}
          // Path(int cap = 0): Path__(cap), distance(0) {}
          inline int get_distance() const noexcept { return cursor + 1; }
          inline bool done() const noexcept { return cursor < 0; }
          inline int get_direction() noexcept { return done() ? POINT : (*this)[cursor--]; }

          int tar_x, tar_y;
          TraceType type_;
          int tar_price = 0;
private:
          // int distance;
          int cursor;      // 指针从后往前
          friend Path __trace_back(int const [N][N], int, int) noexcept;
          friend Path __trace_back_boat(std::vector< std::vector< std::vector<int> > > &direction_, int x, int y, int dir) noexcept;
};

void perfix_action() noexcept;          // 预备动作
void robot_action() noexcept;           // 机器人做出动作
void boat_action() noexcept;            // 船做出动作
void suffix_action() noexcept;

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
                    // int id, take_good, x, y, status;
                    // take_good.1 == 带货/ status.0 == 异常/ status.1 == 正常
                    int id, x, y, goods_num;

                    inline void move(int dir) const noexcept { printf("move %d %d\n", id, dir); }
                    inline void get() const noexcept { printf("get %d\n", id); }
                    inline void pull() const noexcept { printf("pull %d\n", id); }
          };
          struct Boat__ {
                    // int num, pos, status;
                    // pos.-1 == 虚拟点/ status.0 == 运输中/ status.1 == 装货或运输完成/ status.2 == 等待泊位

                    int id, x, y, dir;
                    int goods_num, status;
                    // status.0 == 正常/ status.1 == 恢复/ status.2 == 装载/ dir与机器人Direction一致

                    inline void dept() const noexcept { printf("dept %d\n", id); }
                    inline void berth() const noexcept { printf("berth %d\n", id); }
                    inline void rot(int rot) const noexcept { printf("rot %d %d\n", id, rot - CLOCKWISE); }
                    inline void ship() const noexcept { printf("ship %d\n", id); }
          };
}

/**
 * 扩充地图上的基本元素
*/

// TODO: 
struct Berth: public BaseElem::Berth__ {
          int crt_num = 0;
          std::mutex mtx;
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

          inline void pull() const noexcept;
          inline void move(int) noexcept;
          inline void get() noexcept;
};

struct Boat: public BaseElem::Boat__ {
          int crt_num = 0;              // 当前载货
          bool can_leave = false;       // 是否载货后离开送货
                    // 送货路上是true/ 取货路上是false
          Berth *aim = nullptr;         // 目标泊点
          Path current_path;            // 待走路径

          inline void move() noexcept;
};

/**
 * 扩展的用于存储`机器人`与`泊点`这种包含x、y坐标信息的类型 [基本不会被修改]
*/
namespace BaseElem {
          typedef bool (*cmp) (int x, int y, const Berth &b);
          template <typename T>
          class IfStor__ {
          public:
                    inline bool find(int x, int y, const cmp fn = nullptr) noexcept {
                              return static_cast<T *>(this)->find_(x, y, fn);
                    }
          };

          typedef std::vector<Berth> BerthStor__;
          typedef std::unordered_map<int, Good> GoodStor__;

          class BerthStor: public BerthStor__, public IfStor__<BerthStor> {
          public:
                    template <typename... TArgs>
                    explicit BerthStor(TArgs... args): BerthStor__(std::forward<TArgs>(args)...) {}

                    inline decltype(auto) iter_find(int x, int y, const std::function<bool(int,int,Berth &)> &cmp = [](int x,int y, Berth &berth) {
                              return x == berth.x && y == berth.y;
                    }) {
                              for (auto iter = BerthStor__::begin(); iter != BerthStor__::end(); ++iter)
                              if (cmp(x, y, *iter)) return iter;
                              return BerthStor__::end();
                    }

                    inline decltype(auto) iter_find(int x, int y) noexcept {
                              for (auto iter = BerthStor__::begin(); iter != BerthStor__::end(); ++iter)
                              if ((*iter).x == x && (*iter).y == y) return iter;
                              return BerthStor__::end();
                    }

                    // 接口方法
                    inline bool find_(int x, int y, const cmp fn = nullptr) noexcept {
                              if (fn == nullptr) {
                                        for (auto iter = BerthStor__::begin(); iter != BerthStor__::end(); ++iter)
                                        if ((*iter).x == x && (*iter).y == y) return true;
                                        return false;
                              } else {
                                        for (auto iter = BerthStor__::begin(); iter != BerthStor__::end(); ++iter)
                                        if (fn(x, y, *iter)) return true;
                                        return false;
                              }
                    }
          };

          class GoodStor: public GoodStor__, public IfStor__<GoodStor> {
          public:
                    template <typename... TArgs>
                    explicit GoodStor(TArgs... args): GoodStor__(std::forward<TArgs>(args)...) {}

                    // 货物就只有一个x、y，所以直接返回bool
                    inline bool find_(int key) noexcept { return (GoodStor__::find(key) != GoodStor__::end()); }
                              // 接口方法
                    inline bool find_(int x, int y, const cmp fn = nullptr) noexcept { return find_(as_key(x, y)); }
                    inline decltype(auto) get(int key) noexcept { return GoodStor__::operator[](key); }
                    inline decltype(auto) get(int x, int y) noexcept { return (*this)[as_key(x, y)]; }
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
          typedef std::vector<std::pair<int, int>> PurchasePoint__;
          template <typename T> class PurchasePoint;

          // Base
          struct Base {
                    static inline void init() noexcept {
                              // display(Base init......\n);

                              grid = (char **)malloc(N * sizeof(char *));
                              for (int i = 0; i < N; ++i) grid[i] = (char *)malloc(N * sizeof(char));
                              // grid.resize(N);

                              for (int i = 0; i < N; ++i) scanf("%s", grid[i]);
                              Base::ProcessMap();
                              // Base::DisplayMap();

                              scanf("%d", &berth_num);
                              berths.resize(berth_num);
                              for (int i = 0; i < berth_num; ++i) {
                                        int id; scanf("%d", &id);
                                        scanf("%d%d%d", &berths[id].x, &berths[id].y, &berths[id].loading_speed);
                                        berths[id].id = id;
                              }
                              scanf("%d", &boat_capacity);
                              
                              CHECK_OK(); printf("OK\n"); fflush(stdout);
                    }
                    static inline void dealloc() noexcept {
                              for (int i = 0; i < N; ++i) if (grid[i]) free(grid[i]);
                              if (grid) free(grid);
                    }

                    static BerthStor berths;
                    static int boat_capacity;
                    static char **grid;
                    // static std::vector<std::string> grid;

                    static int robot_num;
                    static int boat_num;
                    static int berth_num;
                    static int goods_num;

                    static PurchasePoint<Robot> robot_purchase_point;
                    static PurchasePoint<Boat> boat_purchase_point;
                    static std::vector<std::pair<int, int>> delivery_point;
          private:
                    static void ProcessMap() noexcept;
                    static void DisplayMap() noexcept;
          };

          // Frame
          struct Frame {
                    static int id, money;
                    static GoodStor goods;
                    static std::vector<Robot> robots;
                    static std::vector<Boat> boats;

                    static int gap_id;  // 与上一次update之间差了几帧
                    
                    static inline void init() noexcept {
                              // display(Frame init......\n);

                              // Frame::robots.resize(ROBOT_NUM);
                              // Frame::boats.resize(BOAT_NUM);
                              Frame::robots.reserve(ROBOT_LIMIT);
                              Frame::boats.reserve(BOAT_LIMIT);

                              // there is no robots and boats......
                    }
                    static inline int update() noexcept;
          };

          /**
           * Purchase部分
          */

          template <typename T>
          class PurchasePoint: public PurchasePoint__ {
          public:
                    template <typename... TArgs>
                    explicit PurchasePoint(TArgs... args): PurchasePoint__(std::forward<TArgs>(args)...)/*, good_num(0), total_price(0)*/ {
                              good_in_range.resize(PurchasePoint__::size(), {0, 0});
                    }
                    
                    // make instance in purchase/ robot or boat
                    bool purchase(int purchase_id) noexcept {
                              if (purchase_id >= PurchasePoint__::size()) {
                                        fprintf(stderr, "Purchae id out of boundary......\n");
                                        return false;
                              }
                              // display(Buy something in purchase id: %d\n, purchase_id);
                              if constexpr (std::is_same_v<T, Robot>) {
                                        if (Frame::money < ROBOT_PRICE) return false;
                                        Base::robot_num++; Frame::money -= ROBOT_PRICE;   // 更新数据
                                        auto &[pur_x, pur_y] = (*this)[purchase_id];
                                        printf("lbot %d %d\n", pur_x, pur_y);
                                        Frame::robots.push_back(Robot{
                                                  (int)Frame::robots.size(), pur_x, pur_y, 0,       // id/ x/ y/ goods_num
                                                  pur_x, pur_y                                      // trace_x/ trace_y
                                        });
                              } else {
                                        display(PURCHASE BOAT......\n);
                                        if (Frame::money < BOAT_PRICE) return false;
                                        Base::boat_num++; Frame::money -= BOAT_PRICE;
                                        printf("lboat %d %d\n", (*this)[purchase_id].first, (*this)[purchase_id].second); 
                              }
                              return true;
                    }

                    struct Record { int num, price; };
                    std::vector<Record> good_in_range;
                    // int good_num, total_price;

                    template <typename... TArgs>
                    inline std::pair<int, int> emplace_back(TArgs&... __args) noexcept {
                              auto ret = PurchasePoint__::emplace_back(std::forward<TArgs>(__args)...);
                              good_in_range.resize(PurchasePoint__::size());
                              return ret;
                    }

                    inline void append_good(const int id, const Good &good) noexcept {
                              // display(Purchase %d append good[%d %d %d %d %d]\n, id, good.x, good.y, good.price, good.purchase_rank, good.live);

                              good_in_range[id].num ++;
                              good_in_range[id].price += good.price;

                              // display(Out append_good(const int, const Good &)\n);
                    }

                    inline void remove_good(const int id, const Good &good) noexcept {
                              good_in_range[id].num --;
                              good_in_range[id].price -= good.price;

                              // 正确性检查
                              record_check__();
                    }

                    inline double find_purchase() const noexcept {
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

          private:
                    inline void record_check__() const noexcept {
                              for (auto &rec : good_in_range) 
                              assert(rec.num >= 0 && rec.price >= 0);
                    }
          };


          void Base::ProcessMap() noexcept {
                    for (int i = 0; i < N; ++i) {
                              for (int j = 0; j < N; ++j) {
                                        if (grid[i][j] == 'R') robot_purchase_point.emplace_back(i, j);
                                        else if (grid[i][j] == 'S') boat_purchase_point.emplace_back(i, j);
                                        else if (grid[i][j] == 'T') delivery_point.emplace_back(i, j);
                              }
                    }
          }

          void Base::DisplayMap() noexcept {
                    display(Robot purchase points:\n);
                    for (auto &&[x, y] : Base::robot_purchase_point) 
                              display([x: %d y: %d]\n, x, y);
                    
                    display(Boat purchasr points:\n);
                    for (auto &&[x, y] : Base::boat_purchase_point) {
                              display([x: %d y: %d]\n, x, y);
                    }
          }

          /**
           * Base部分
          */

          BerthStor Base::berths = BerthStor();
          int Base::boat_capacity = 0;
          char **Base::grid = NULL;

          int Base::robot_num = 0;
          int Base::boat_num = 0;
          int Base::berth_num = 0;
          int Base::goods_num = 0;

          PurchasePoint<Robot> Base::robot_purchase_point = PurchasePoint<Robot>();
          PurchasePoint<Boat> Base::boat_purchase_point = PurchasePoint<Boat>();
          std::vector<std::pair<int, int>> Base::delivery_point = std::vector<std::pair<int, int>>();

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
          int Frame::update() noexcept {
                    display(Frame update......\n);

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
                                                  auto &pur = Base::robot_purchase_point[i];
                                                  int dis_ = std::abs(pur.first - x) + std::abs(pur.second - y);
                                                  if (dis_ < distance_) {
                                                            distance_ = dis_;
                                                            pur_id = i;
                                                  }
                                        }
                                        assert(pur_id != -1);         // 检查一定是找到了一个位置

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
                    // check the number of robot is right

                    // display(TRACE::: Robot num %d and REAK::: Robot num %d...\n, robot_num, Base::robot_num);
                    // assert(robot_num == Base::robot_num);
                    Base::robot_num = robot_num;
                    
                    robots.resize(robot_num);     // 确保有足够的位置
                    for (int i = 0; i < Base::robot_num; ++i) {

                              // 机器人信息来的顺序保持为 0 - n ？
                              int id;
                              scanf("%d", &id);
                              // scanf("%d%d%d%d", &robots[i].id, &robots[i].goods_num, &robots[i].x, &robots[i].y);
                              robots[id].id = id;
                              scanf("%d%d%d", &robots[id].goods_num, &robots[id].x, &robots[id].y);
                    }

                    int boat_num;
                    scanf("%d", &boat_num);
                    // check......
                    assert(boat_num == Base::boat_num);
                    boats.resize(boat_num);       // 确保有足够的位置
                    for (int i = 0; i < Base::boat_num; ++i) {
                              scanf("%d%d%d%d%d%d", &boats[i].id, &boats[i].goods_num, &boats[i].x, &boats[i].y, &boats[i].dir, &boats[i].status);
                              display("get message about boat %d [x %d, y, %d, dir %d, status %d]"\n, boats[i].id, boats[i].x, boats[i].y, boats[i].dir, boats[i].status);
                    }

                    // char okk[100];
                    // scanf("%s", okk);
                    return id;
          }
}

/**
 * 扩充的Base以及Frame
 * 把原来的基本数据和扩充的新数据隔离开，除非必须在原来的Base和Frame上进行更改，或者这样扩展影响效率
*/

struct MyBase: public BaseElem::Base {
          // static int ***locks_;
          static std::vector< std::vector< std::vector<int> > > locks_;
          static std::vector< std::vector<int> > zlocks_;

          static inline void init() noexcept {
                    Base::init();
                    // int [N][N][ROBOT_NUM]
                    locks_.resize(N, std::vector< std::vector<int> >(N, std::vector<int>(ROBOT_LIMIT, 0)));
                    zlocks_.resize(N,std::vector<int>(N, 0));
          }
};
// int ***MyBase::locks_ = NULL;
std::vector< std::vector< std::vector<int> > > MyBase::locks_ = std::vector< std::vector< std::vector<int> > >();
std::vector< std::vector<int> > MyBase::zlocks_ = std::vector< std::vector<int> >();

struct MyFrame: public BaseElem::Frame {
          static int clock_for_margin_func;
          static int clock_id, price_count;
          static int need_robot, need_boat;
          static double margin_growth;
          static int router_times;

          static int update() noexcept;
          static inline void init() noexcept { 
                    Frame::init(); 

                    MyFrame::clock_id = MyFrame::id;
          }

          // 购买的动作，是否购买机器人或者船......
          static inline void purchase_action() noexcept;
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
          void init() { MyBase::init(), MyFrame::init(); }
          inline int Run() {
                    if (MyFrame::update() == -1) return -1;
                    puts("OK"); fflush(stdout);
                    return 0;
          }
          void free() { MyBase::dealloc(); }

          Loop() { init(); }
          ~Loop() { display(LOOP EXECUTE OVER!\n); free(); }
};

int main() {
          for (Loop self;;)
          if (self.Run() == -1) return 0;
}


int MyFrame::update() noexcept {
          if (Frame::update() == -1) return -1;

          // 扩充的update信息
          // ...

          // 准备动作/ 机器人动作/ 船最后动
          perfix_action();
          
          std::vector<std::thread> parallel_tasks(2);
          display(Create tasks......\n);
          parallel_tasks[0] = std::thread(robot_action);
          parallel_tasks[1] = std::thread(boat_action);

          // robot_action();
          // boat_action();
          
          display(Tasks join......\n);
          for (int i = 0; i < 2; ++i) parallel_tasks[i].join();
                    
          display(Tasks run over......\n);
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
void MyFrame::purchase_action() noexcept {
          // display(Purchase action......\n);
          if (need_boat > 0 && MyBase::boat_num < BOAT_LIMIT) {
                    int id = std::rand() % MyBase::boat_purchase_point.size();
                    display(Buy a boat purchase id %d......\n, id);
                    if (MyBase::boat_purchase_point.purchase(id)) need_boat--;
          }

          if (need_robot > 0 && MyBase::robot_num < ROBOT_LIMIT) {
                    int id = MyBase::robot_purchase_point.find_purchase();
                    display(Buy a robot purchase id %d......\n, id);
                    if (MyBase::robot_purchase_point.purchase(id)) need_robot--;
          }
}

/**
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * == == == == == == == == == == == == == == == == == == == == == == == 算 法 的 具 体 实 现 == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
*/

// 上是x-1，下是x+1，右是y+1，左是y-1
Path __trace_back(int const map[N][N], int x, int y) noexcept {
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
          }
          // ret.distance = distance;
          ret.cursor = ret.size() - 1;
          return ret;
}

/**
 * 筛选路径，只保留一条路径并且对路径进行避免碰撞的操作
 *        ::: 如果不在规划路径时进行碰撞规避，那就只筛选出一条路径
*/
inline Path __sift_and_lock(std::vector<Path> &path) noexcept {
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
inline bool __check_berth(int x, int y) noexcept { return (MyBase::grid[x][y] == 'B'); }
inline bool __check_good(int x, int y) noexcept { return MyFrame::goods.find_(x, y); }

typedef bool (*check_can_move) (char &);
inline bool __both_robot_boat_move(char &c) noexcept { return c == 'C' || c == 'c'; }
inline bool __robot_can_move(char &c) noexcept { return c == '.' || c == '>' || c == 'R' || c == 'B' || __both_robot_boat_move(c); }

std::set<int> seen;
// 寻找路径，默认只找最近的
const /*std::vector<Path>*/ Path router_dij(Robot &robot, size_t cap = 1) noexcept {
          auto &map_ = MyBase::grid;
          // 加锁的操作转移到__sift_and_lock()中，所以这里设置不可修改locks_
          const auto &locks_ = MyBase::locks_;

          check_position __check = robot.goods_num ? (__check_berth) : (__check_good);
          check_can_move __can_move = __robot_can_move;

          int trace_[N][N];             
          int distance_[N][N];          
          for (int i = 0; i < N; ++i)
          for (int j = 0; j < N; ++j) {
                    trace_[i][j] = -1;
                    distance_[i][j] = INT_MAX;
          }

          trace_[robot.x][robot.y] = POINT;
          distance_[robot.x][robot.y] = 0;


          seen.clear();
          auto ret = std::vector<Path>();
          auto wait_ = std::set<Position>(); wait_.emplace(robot.x, robot.y);

          CYCLE__: while (wait_.size()) {
                    std::set<Position> buff_wait;
                    for (auto &pos : wait_) {
                              auto [x, y] = pos;
                              if (__check(x, y) && seen.find(as_key(x, y)) == seen.end()) {
                                        display(Find target ? %d\n, MyFrame::goods.find_(as_key(x, y)));

                                        // 可以容忍的距离/ 去泊点无所谓距离/ 去货物只能容忍货物的存在帧
                                        int tolance_ =  robot.goods_num ? -1 : MyFrame::goods[as_key(x, y)].live;
                                        display(tolance_: %d and distance_: %d\n, tolance_, distance_[x][y]);
                                        if (tolance_ == -1 || distance_[x][y] <= tolance_) {
                                                  // display(Find target_......\n);
                                                  auto path = __trace_back(trace_ , x, y);
                                                  path.type_ = robot.goods_num ? BERTH : GOOD;
                                                  path.tar_price = MyFrame::goods[as_key(x, y)].price;

                                                  ret.emplace_back(path);
                                                  seen.emplace(as_key(x, y));
                                                  if (ret.size() >= cap) goto RET__;
                                        } else {
                                                  display(Beyond the distance can be tolerated! No path is detected.\n);
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
                    wait_ = buff_wait;
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
Path __trace_back_boat(std::vector< std::vector< std::vector<int> > > &direction_, int x, int y, int dir) noexcept {
          auto ret = Path();
          // 确定船的Path中每一个方向
          auto confirm_dir_ = [&](int last_dir, int crt_dir) {
                    if (last_dir == crt_dir) return SHIP;
                    switch (last_dir) {
                              case RIGHT: return crt_dir == DOWN ? CLOCK : ANTI;
                              case DOWN: return crt_dir == LEFT ? CLOCK : ANTI;
                              case LEFT: return crt_dir == UP ? CLOCK : ANTI;
                              case UP: return crt_dir == RIGHT ? CLOCK : ANTI;
                              default: {
                                        display(::: [dir %d]Unknown direction of boat!......\n, last_dir);
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
                    for (auto &[dx, dy, ddir_] : to_next_) {
                              if (ddir_ != cdir_) continue;
                              x -= dx, y -= dy;
                              cdir_ = dir_;
                              return;
                    }
                    display([ERROR] Can not trace back to the front point!\n);
          };

          // 不是POINT就代表存在上一个状态
          while (direction_[x][y][dir] != POINT) {
                    display(TRACE::: x %d y %d dir %d\n, x, y, dir);

                    ret.emplace_back(confirm_dir_(direction_[x][y][dir], dir));
                    back_point_(x, y, dir, direction_[x][y][dir]);
          }
          display(TRACE::: Terminal x %d y %d\n, x, y);

          // if (ret.size()) ret.erase(ret.begin());
          ret.cursor = ret.size() - 1;
          return ret;
}

// args 可以为任何类型参数/ 自定义函数可能需要不同类型的参数
typedef bool (*check_position_boat) (int x, int y, int dir, void *);
// 默认方法/ TODO: 
inline bool __check_position_boat_default(int x, int y, int dir, void *args) {
          // 最简单的就是判断 核心点 到达K点
          return MyBase::grid[x][y] == 'K';
}
inline bool __check_position_boat_1 (int x, int y, int dir, void *args) {
          return false;
}

// 船寻路
const Path 
router_boat(
          Boat &boat, 
          const check_position_boat __check_position = __check_position_boat_default, 
          void *args = NULL   // 比如传入一个Berth，判断是否到达这个berth
                              // 或者传入一个id，判断是否到达指定id的berth
) noexcept {
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
                              case UP: ret = (map_[x - 2][y] == '~' || map_[x - 2][y] == 'c' || map_[x - 2][y + 1] == '~' || map_[x - 2][y + 1] == 'c') ? 2 
                                        : 1;
                                        break;
                              case DOWN: ret = (map_[x + 2][y] == '~' || map_[x + 2][y] == 'c' || map_[x + 2][y - 1] == '~' || map_[x + 2][y - 1] == 'c') ? 2 
                                        : 1;
                                        break;
                              case LEFT: ret = (map_[x][y - 2] == '~' || map_[x][y - 2] == 'c' || map_[x - 1][y  - 2] == '~' || map_[x - 1][y - 2] == 'c') ? 2 
                                        : 1;
                                        break;
                              case RIGHT: ret = (map_[x][y + 2] == '~' || map_[x][y + 2] == 'c' || map_[x + 1][y + 2] == '~' || map_[x + 1][y + 2] == 'c') ? 2 
                                        : 1;
                                        break;
                    }
                    return ret;
          };

          Path path;
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
                                        display(KPOINT::: Find K point [%d %d]......\n, x, y);
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
          return Path(); // __trace_back_boat(direction_, 0, 0, POINT);      // 空/ 没有找到路径
}

// 在机器人和船行动前先进行的操作
void perfix_action() noexcept {
          // display(Perfix action......\n);

          MyFrame::router_times = 0;    // 找路次数清零
          // 检查机器人位置的正确性
          for (auto &robot : MyFrame::robots) {
                    // assert(robot.x == robot.trace_x);
                    // assert(robot.y == robot.trace_y);

                    // display(Robot %d: [x: %d and trace_x: %d] [y: %d and trace_t: %d]\n, robot.id, robot.x, robot.trace_x, robot.y, robot.trace_y);

                    if (robot.x != robot.trace_x || robot.y != robot.trace_y) {
                              display(::: 机器人 %d 发生了碰撞[frame id %d]\n, robot.id, MyFrame::id);
                              // 归位
                              robot.collision = true;
                              robot.trace_x = robot.x;
                              robot.trace_y = robot.y;
                    }
          }
}
// std::vector<Robot *> after_move;
// 机器人的操作/ 为每个机器人寻路/ 每个机器人移动或拿起放下货物/ [TODO: 需要修改为多线程并发操作 X]
std::vector<std::thread> tasks_;
void robot_action() noexcept {
          display(Robot action::: Robot size %ld[ %d ]......\n, MyFrame::robots.size(), MyBase::robot_num);

          /*auto robot_task_ = [&](Robot *robot) {
                    if (robot->collision) robot->current_path.clear(), robot->stack_.clear(), robot->collision = false;  // 碰撞之后重新寻路

                    // 机器人寻路，一帧中最多找ROUTER_LIMIT_PER_FRAME次路
                    if (robot->current_path.empty()) {
                              auto router = robot->goods_num ? router_dij(*robot) : router_dij(*robot, CMP_TARGET_NUM);

                              // 目前就一条路径
                              robot->current_path = std::move(router);
                              if (robot->current_path.type_ == GOOD) {
                                        int key = as_key(robot->current_path.tar_x, robot->current_path.tar_y);
                                        MyFrame::goods.erase(key);
                              }
                              
                    }
          };

          tasks_.resize(MyBase::robot_num);
          for (int i = 0; i < MyBase::robot_num; ++i) tasks_[i] = std::thread(robot_task_, &MyFrame::robots[i]);
          display(::: Parallel robot task count %ld created......\n, tasks_.size());

          try {
                    for (int i = 0; i < MyBase::robot_num; ++i) tasks_[i].join();
          } catch (std::exception e) {
                    display(::: ERROR::: exception from robot task: %s\n, e.what());
          }
          display(::: Parallel robot task done......\n);*/

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

                              display(::: Target price::: %d !!!\n, router.tar_price);

                              // 目前就一条路径
                              robot.current_path = std::move(router);
                              if (robot.current_path.type_ == GOOD) {
                                        int key = as_key(robot.current_path.tar_x, robot.current_path.tar_y);
                                        MyFrame::goods.erase(key);
                              }
                              
                    }


                    if (robot.current_path.empty()) {
                              /*if (robot.goods_num == 0) {
                                        robot.get();
                                        MyFrame::goods.erase(as_key(robot.x, robot.y));
                              } else robot.pull();*/
                              continue;
                    }
                    // 机器人移动
                    
                    auto &path = robot.current_path;

                    // 当存在临时路径，先走临时路径
                    if (robot.stack_.size()) {
                              auto dir_ = robot.stack_.back();
                              robot.stack_.pop_back();
                              
                              robot.move(dir_);
                    } else robot.move(path.get_direction());
                    
                    if (path.done() && robot.stack_.empty()) { //路径走完了
                              display(::: Robot goods_num: %d\n, robot.goods_num);
                              
                              if (robot.goods_num) robot.pull();
                              else robot.get();
                              robot.current_path.clear();   // 走完整条路径，忘掉之前的路径
                    }
          }

          display(::: Robot action done...\n);
}

// 船的操作/ 分配泊点/ 装卸货物/ 期望[102 70] 实际[104 70]
void boat_action() noexcept {
          display(Boat action......\n);
          display(::: Boat number %ld......\n, MyFrame::boats.size());

          for (auto &boat : MyFrame::boats) {
                    display(::: Boat position [%d %d]......\n, boat.x, boat.y);

                    if (boat.status == 0) { // 正常状态 
                              if (boat.current_path.empty()) {
                                        display(TRACE::: router from position %d %d...\n, boat.x, boat.y);
                                        boat.current_path = router_boat(boat);  // 寻找一个最近的泊点
                                        display(Boat find a path with distance %d\n, boat.current_path.get_distance());
                              }

                              if (boat.current_path.empty()) continue;
                              // DEBUG
                              // boat.rot(ANTI);

                              auto &path = boat.current_path;
                              if (!path.done()) {
                                        int dir_ = path.get_direction();
                                        display(TRACE::: dir is %d...\n, dir_);
                                        if (dir_ == SHIP) boat.ship();
                                        else boat.rot(dir_);
                              } else {
                                        if (boat.can_leave == false) boat.berth();
                              }

                    } else if (boat.status == 1) {          // 恢复状态
                              display(TRACE::: Boat %d is recovering (distance %d).\n, boat.id, boat.current_path.get_distance());
                    } else {  // 装载状态
                              // boat.dept();        // 看看能去哪
                              display(Boat %d is not in normal status.\n, boat.id);
                    }
          }

          display(::: Boat action done......\n);
}

void suffix_action() noexcept {
          
}


/**
 * 扩展的Robot动作
*/

long total_price_ = 0;
void Robot::get() noexcept {
          Robot__::get();
          
          current_price = current_path.size() ? current_path.tar_price : (
                    MyFrame::goods.find_(x, y) ? MyFrame::goods[as_key(x, y)].price : 0
          );

          total_price_ += current_price;
          display(::: Pull::: total price %ld .\n, total_price_);
}

void Robot::pull() const noexcept {
          Robot__::pull();

          // 更新MyFrame判断是否增加机器人的数据
          MyFrame::price_count += current_price;
          // 当记时器结束的时候根据 上一次增加机器人后 每帧平均价值判断是否应该增加机器人/ 现在的逻辑是只要大于没增加的就再次增加，
                    // 可以设置一个阈值，当增加了多少才选择再次增加机器人
          if(-- MyFrame::clock_for_margin_func == 0) {
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
                    display(DETAIL ::: MAP_CHAR:: %c CMP_FUNC(%d %d Berth[%d %d])\n, MyBase::grid[x][y], x, y, berth.x, berth.y);
                    return std::abs(x - berth.x) < 9 && std::abs(y - berth.y) < 9;
          });
          if (ret != MyBase::berths.end()) {
                    (*ret).crt_num += 1;
                    display(INFO ::: Berth %d current_number is %d.\n, (*ret).id, (*ret).crt_num);
          }
          else {
                    display(ERROR ::: Can not find a berth to pull good......\n);
          }
}

inline void __force_lock(int x, int y) noexcept {
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
inline void __force_unlock(int x, int y) noexcept { MyBase::zlocks_[x][y] = 0; }

// 用于选择优先避让方向
std::vector<int> direction_list {LEFT, RIGHT, UP, DOWN};
void Robot::move(int dir) noexcept {
          if (dir == POINT) {
                    display(POINT direction );
          }
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

                              if (MyBase::grid[ctrace_x][ctrace_y] == '#' || MyBase::grid[ctrace_x][ctrace_y] == '*' || MyBase::grid[ctrace_x][ctrace_y] == '~' ||
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

// TODO: 船有体积，所以移动还是寻路都有限制
void Boat::move() noexcept {
          Boat__::ship();

}