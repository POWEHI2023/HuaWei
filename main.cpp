
/**
 * 当前实现策略：
 *        开始时init初始化Base和Frame
 *        每一帧首先调用update更新参数
 *        然后按照perfix、robot、boat、purchase、suffix的顺序执行动作
 *        
 *        ::: 关于购买
 *        目前在更新新货物信息时，会把货物归类到离他最近的purchase
 *        购买机器人时，判断哪一个purchase范围内货物价值密度最高，在最高的purchase点购买机器人
 * 
 *        ::: 关于是否购买机器人
 *        初始时购买机器人和船的数量都是1
 *        当执行到purchase_action时判断是否need_robot或者need_boat大于0，当大于0时执行购买指令
 *        当机器人pull货物到泊点时更新送货的价值曲线，是否因为上一次的增加机器人而导致`帧数/搬运价值`增加 [相关逻辑还没有实现/ 实现了]
 *        如果增加的话说明可以再次购买机器人，最多购买ROBOT_LIMIT个机器人
 * 
 *        ::: 关于是否购买船
 *        购买船的逻辑还没有写，只有初始购买的1艘船/ 部分实现
 * 
 *        ::: 关于机器人的移动碰撞检测
 *        由于没有相关状态检测机器人是否异常/ 碰撞不会导致停顿
 *        为每个机器人设置trace_x和trace_y，每次移动更新两个的值
 *        每一帧刷新后如果机器人移动正常trace_x和trace_y应该等于x和y，如果不等于则发生了碰撞
 * 
 *        目前发生碰撞会忘掉之前的路重新找路
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

#define ROBOT_LIMIT 20        // 设置一个最大量
#define BOAT_LIMIT 10         // 设置一个最大量

#define ROUTER_LIMIT_PER_FRAME 5        // 每帧最多找几次路

#define display(msg, ...) ({ fprintf(stderr, #msg, ##__VA_ARGS__); })
#define CHECK_OK() ({ char okk[100]; scanf("%s", okk); })

typedef std::tuple<int, int> Position;
enum Direction {RIGHT = 0, LEFT, UP, DOWN, POINT};
constexpr int CLOCKWISE = 1 << 5;
enum Rotation {CLOCK = CLOCKWISE, ANTI};

#define as_key(x, y) ((x << 10) | y)
#define as_position(key) ({ {key >> 10, key & 0x3FF} })

/**
 * 提前声明类型方便后续访问
*/

typedef std::vector<int> Path__;
class Path;

enum TraceType { GOOD, BERTH, None };

// 用来构造路径
Path __trace_back(int const map[N][N], int x, int y, int & distance) noexcept;

/**
 * 路径指针，用来移动
 * tar参数为目标位置
 * TraceType为寻路的目标类型
 * distance为路径距离
*/
class Path: public Path__ {
public:
          template <typename... TArgs>
          Path(TArgs... args): Path__(std::forward<TArgs>(args)...), distance(0), cursor(-1), tar_x(0), tar_y(0), type_(None) {}
          // Path(int cap = 0): Path__(cap), distance(0) {}
          int inline get_distance() const noexcept { return distance - Path__::size() + cursor + 1; }

          int cursor;      // 指针从后往前
          int tar_x, tar_y;
          TraceType type_;
private:
          int distance;
          friend Path __trace_back(int const [N][N], int, int, int &) noexcept;
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

struct Berth: public BaseElem::Berth__ {
          int crt_num = 0;

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

          inline void pull() const noexcept;
          inline void move(int) noexcept;
          inline void get() noexcept;
};

struct Boat: public BaseElem::Boat__ {
          int crt_num = 0;
          bool can_leave = false;
          Berth *aim = nullptr;

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

                    /** 
                     * 寻找一个符合条件的值，是否符合条件由参数cmp判断
                     *        自定义函数: cmp传入的参数最后一个需要为Berth &
                     * 泊点是一整片B，所以可能要判断是否在范围内
                     * 
                     * find(x, y, [](int x, int y, Berth &berth) {
                     *        return (berth.x <= x && berth.x + 3 > x && berth.y >= y && berth.y + 3 < y);
                     * }); (例子)
                    */
                    template <typename... Args, typename T>
                    inline decltype(auto) iter_find(Args... args, const T &cmp) {
                              for (auto iter = BerthStor__::begin(); iter != BerthStor__::end(); ++iter)
                              if (cmp(std::forward(args)..., *iter)) return iter;
                              return BerthStor__::end();
                    }
                    /** 
                     * 特化的find方法
                     * BerthStor::iterator iter = 实例.find(x, y);
                     * 当匹配到x、y与某个泊点重合，通过(*iter)得到泊点的引用
                     * 当没有匹配到是，iter == 实例.end();
                    */
                    inline decltype(auto) iter_find(int x, int y) {
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
                              for (int i = 0; i < N; ++i) free(grid[i]);
                              free(grid);
                    }

                    static BerthStor berths;
                    static int boat_capacity;
                    static char **grid;

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
                    // display(Frame update......\n);

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
                    }

                    int robot_num;
                    scanf("%d", &robot_num);
                    // check the number of robot is right
                    assert(robot_num == Base::robot_num);
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

          static inline void init() noexcept {
                    Base::init();
                    // int [N][N][ROBOT_NUM]
                    locks_.resize(N, std::vector< std::vector<int> >(N, std::vector<int>(ROBOT_LIMIT, 0)));
          }
};
// int ***MyBase::locks_ = NULL;
std::vector< std::vector< std::vector<int> > > MyBase::locks_ = std::vector< std::vector< std::vector<int> > >();

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
int MyFrame::need_robot { 1 };
int MyFrame::need_boat { 1 };
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
          ~Loop() { free(); }
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
          perfix_action(), robot_action(), boat_action();

          // 购买动作
          purchase_action(), suffix_action();

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
          if (need_robot > 0 && MyBase::robot_num < ROBOT_LIMIT) {
                    int id = MyBase::robot_purchase_point.find_purchase();
                    display(Buy a robot purchase id %d......\n, id);
                    if (MyBase::robot_purchase_point.purchase(id)) need_robot--;
          }

          if (need_boat > 0 && MyBase::boat_num < BOAT_LIMIT) {
                    int id = std::rand() % MyBase::boat_purchase_point.size();
                    if (MyBase::boat_purchase_point.purchase(id)) need_boat--;
          }
}

/**
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * == == == == == == == == == == == == == == == == == == == == == == == 算 法 的 具 体 实 现 == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
 * == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == 
*/

// 上是x-1，下是x+1，右是y+1，左是y-1
Path __trace_back(int const map[N][N], int x, int y, int &distance) noexcept {
          Path ret;
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
          ret.distance = distance;
          ret.cursor = ret.size() - 1;
          ret.tar_x = x; ret.tar_y = y;
          return ret;
}

typedef bool (*check_position) (int, int);
inline bool __check_berth(int x, int y) noexcept { return (MyBase::grid[x][y] == 'B'); }
inline bool __check_good(int x, int y) noexcept {
          // Storage<GoodStor> &stor = MyFrame::goods;
          return MyFrame::goods.find_(x, y);
}

// 船和机器人都可以走的地方
inline bool __both_robot_boat_move(char &c) noexcept {
          return c == 'C' || c == 'c';
}
// 机器人可以走的路
inline bool __robot_can_move(char &c) noexcept {
          return c == '.' || c == '>' || c == 'R' || c == 'B' || __both_robot_boat_move(c);
}
// 船可以走的路
inline bool __boat_can_move(char &c) noexcept {
          return c == '*' || c == '~' || c == 'S' || c == 'K' || c == 'T' || __both_robot_boat_move(c);
}

std::set<int> seen;
// 寻找路径，默认只找最近的
const std::vector<Path> router_dij(const Robot &robot, size_t cap = 1) noexcept {
          auto &map_ = MyBase::grid;
          auto &locks_ = MyBase::locks_;
          check_position check_ = robot.goods_num ? (__check_berth) : (__check_good);
          
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
          auto __traverse = [&]() -> int {
                    auto buff_wait = std::set<Position>();
                    CYCLE__: for (auto &pos : wait_) {
                              auto [x, y] = pos;
                              if (check_(x, y) && seen.find(as_key(x, y)) == seen.end()) {
                                        display(Find target ? %d\n, MyFrame::goods.find_(as_key(x, y)));

                                        // 可以容忍的距离/ 去泊点无所谓距离/ 去货物只能容忍货物的存在帧
                                        int tolance_ = robot.goods_num ? -1 : MyFrame::goods[as_key(x, y)].live;
                                        display(tolance_: %d and distance_: %d\n, tolance_, distance_[x][y]);

                                        if (tolance_ == -1 || distance_[x][y] <= tolance_) {
                                                  // display(Find target_......\n);
                                                  auto path = __trace_back(trace_ , x, y, distance_[x][y]);
                                                  path.type_ = robot.goods_num ? BERTH : GOOD;

                                                  ret.emplace_back(path);
                                                  seen.emplace(as_key(x, y));
                                                  if (ret.size() < cap) return 0;
                                                  else return -1;
                                        } else {
                                                  display(Beyond the distance can be tolerated! No path is detected.\n);
                                                  return -1;
                                        }
                              }

                              int distance = distance_[x][y] + 1;

                              if (__robot_can_move(map_[x-1][y]) && distance < distance_[x-1][y]) {
                                        trace_[x-1][y] = UP;
                                        distance_[x-1][y] = distance;
                                        buff_wait.emplace(x-1, y);
                              }
                              if (__robot_can_move(map_[x+1][y]) && distance < distance_[x+1][y]) {
                                        trace_[x+1][y] = DOWN;
                                        distance_[x+1][y] = distance;
                                        buff_wait.emplace(x+1, y);
                              }
                              if (__robot_can_move(map_[x][y-1]) && distance < distance_[x][y-1]) {
                                        trace_[x][y-1] = LEFT;
                                        distance_[x][y-1] = distance;
                                        buff_wait.emplace(x, y-1);
                              }
                              if (__robot_can_move(map_[x][y+1]) && distance < distance_[x][y+1]) {
                                        trace_[x][y+1] = RIGHT;
                                        distance_[x][y+1] = distance;
                                        buff_wait.emplace(x, y+1);
                              }
                    }
                    if (buff_wait.empty()) return 0;
                    wait_ = std::move(buff_wait);
                    goto CYCLE__;
                    
          } ();
          return ret;
}

// 在机器人和船行动前先进行的操作
void perfix_action() noexcept {
          // display(Perfix action......\n);

          MyFrame::router_times = 0;    // 找路次数清零
          // 检查机器人位置的正确性
          for (auto &robot : MyFrame::robots) {
                    // assert(robot.x == robot.trace_x);
                    // assert(robot.y == robot.trace_y);
                    display(Robot %d: [x: %d and trace_x: %d] [y: %d and trace_t: %d]\n, robot.id, robot.x, robot.trace_x, robot.y, robot.trace_y);
                    if (robot.x != robot.trace_x || robot.y != robot.trace_y) {
                              display(::: 机器人 %d 发生了碰撞[frame id %d]\n, robot.id, MyFrame::id);
                              // 归位
                              robot.collision = true;
                              robot.trace_x = robot.x;
                              robot.trace_y = robot.y;
                    }
          }
}
// 机器人的操作/ 为每个机器人寻路/ 每个机器人移动或拿起放下货物/ [TODO: 需要修改为多线程并发操作]
void robot_action() noexcept {
          // display(Robot action......\n);

          /*for (auto &robot : MyFrame::robots) {
                    // 随机移动测试......
                    robot.move(std::rand() % POINT);
          }*/

          for (auto &robot : MyFrame::robots) {
                    if (robot.collision) robot.current_path.clear(), robot.collision = false;  // 碰撞之后重新寻路
                    // 机器人寻路，一帧中最多找ROUTER_LIMIT_PER_FRAME次路
                    if (robot.current_path.empty() && MyFrame::router_times < ROUTER_LIMIT_PER_FRAME) {
                              auto router = router_dij(robot);
                              MyFrame::router_times++;      // 找路次数加一
                              if (router.empty()) continue;

                              // 目前就一条路径
                              robot.current_path = std::move(router[0]);
                              if (robot.current_path.type_ == GOOD) {
                                        MyFrame::goods.erase(as_key(robot.current_path.tar_x, robot.current_path.tar_y));
                              }
                              
                    }

                    if (robot.current_path.empty()) continue;
                    // 机器人移动
                    
                    auto &path = robot.current_path;
                    
                    // if (path.cursor >= 0) { // DEBUG时候必须有if，因为不会在没路以后再找路
                    robot.move(path[path.cursor--]);
                    /*} else*/ 
                    if (path.cursor < 0) { //路径走完了
                              // 随机移动测试......
                              // robot.move(std::rand() % POINT);
                              display(::: Robot goods_num: %d\n, robot.goods_num);
                              
                              if (robot.goods_num) robot.pull();
                              else robot.get();
                              robot.current_path.clear();   // 走完整条路径，忘掉之前的路径
                    }
          }

}
// 船的操作/ 分配泊点/ 装卸货物
void boat_action() noexcept {
          // display(Boat action......\n);

}

void suffix_action() noexcept {
          
}

void Robot::get() noexcept {
          Robot__::get();
          
          int key = as_key(Robot::trace_x, Robot::trace_y);
          if (MyFrame::goods.find_(key)) {
                    auto &good = MyFrame::goods.get(key);
                    current_price = good.price;
          }
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
}

void Robot::move(int dir) noexcept {
          Robot__::move(dir);
          switch (dir) {
                    case UP: trace_x--; break;
                    case DOWN: trace_x++; break;
                    case LEFT: trace_y--; break;
                    case RIGHT: trace_y++; break;
                    default: {
                              display(::: Unknown direction[%d]......\n, dir);
                    }
          }
}