// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <std_msgs/msg/float32.hpp>
// #include <cmath>
// #include <vector>
// #include <string>
// #include <algorithm>

// class ScanNoiseMonitor : public rclcpp::Node {
// public:
//   ScanNoiseMonitor() : Node("scan_noise_monitor")
//   {
//     scan_topic_   = this->declare_parameter<std::string>("scan_topic", "/scan");
//     spike_th_     = this->declare_parameter<double>("spike_threshold", 0.20);
//     lonely_th_    = this->declare_parameter<double>("lonely_threshold", 0.30);
//     angles_deg_   = this->declare_parameter<std::vector<int>>("angles_deg", {90,180,270});

//     pub_sigma_    = this->create_publisher<std_msgs::msg::Float32>("/scan_metrics/sigma", 10);
//     pub_badpct_   = this->create_publisher<std_msgs::msg::Float32>("/scan_metrics/bad_pct", 10);
//     pub_spikepct_ = this->create_publisher<std_msgs::msg::Float32>("/scan_metrics/spikes_pct", 10);
//     pub_lonelypct_= this->create_publisher<std_msgs::msg::Float32>("/scan_metrics/lonely_pct", 10);

//     for (int a : angles_deg_) {
//       angle_pubs_.push_back({
//         a,
//         this->create_publisher<std_msgs::msg::Float32>("/scan_angle/deg" + std::to_string(a), 10)
//       });
//     }

//     sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       scan_topic_, rclcpp::SensorDataQoS(),
//       std::bind(&ScanNoiseMonitor::cbScan, this, std::placeholders::_1));

//     RCLCPP_INFO(get_logger(), "Listening %s ; angles=%s",
//       scan_topic_.c_str(), vecToStr(angles_deg_).c_str());
//   }

// private:
//   struct AnglePub { int deg; rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub; };

//   void cbScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//   {
//     const auto & ranges = msg->ranges;
//     const size_t n = ranges.size();
//     if (n == 0) return;

//     // Collect valid values & basic stats
//     size_t bad = 0;
//     double sum = 0.0, sum2 = 0.0;
//     double vmin = std::numeric_limits<double>::infinity();
//     double vmax = 0.0;
//     size_t valid_count = 0;

//     for (float v : ranges) {
//       if (std::isfinite(v)) {
//         valid_count++;
//         sum  += v;
//         sum2 += double(v) * double(v);
//         if (v < vmin) vmin = v;
//         if (v > vmax) vmax = v;
//       } else {
//         bad++;
//       }
//     }
//     if (valid_count == 0) return;

//     const double mean = sum / double(valid_count);
//     const double var  = std::max(0.0, (sum2 / double(valid_count)) - mean*mean);
//     const double sigma = std::sqrt(var);
//     const double badpct = 100.0 * double(bad) / double(n);

//     // Spikes: |r[i]-r[i-1]| > spike_th_
//     size_t spikes = 0;
//     for (size_t i = 1; i < n; ++i) {
//       const float a = ranges[i], b = ranges[i-1];
//       if (std::isfinite(a) && std::isfinite(b)) {
//         if (std::fabs(double(a) - double(b)) > spike_th_) spikes++;
//       }
//     }
//     const double spikepct = 100.0 * double(spikes) / double(n > 1 ? (n-1) : 1);

//     // Lonely: |r[i] - 0.5*(r[i-1]+r[i+1])| > lonely_th_
//     size_t lonely = 0;
//     for (size_t i = 1; i + 1 < n; ++i) {
//       const float l = ranges[i-1], c = ranges[i], r = ranges[i+1];
//       if (std::isfinite(l) && std::isfinite(c) && std::isfinite(r)) {
//         const double neigh = 0.5 * (double(l) + double(r));
//         if (std::fabs(double(c) - neigh) > lonely_th_) lonely++;
//       }
//     }
//     const double lonelypct = 100.0 * double(lonely) / double(n > 2 ? (n-2) : 1);

//     publishFloat(pub_sigma_, float(sigma));
//     publishFloat(pub_badpct_, float(badpct));
//     publishFloat(pub_spikepct_, float(spikepct));
//     publishFloat(pub_lonelypct_, float(lonelypct));

//     // Publish selected angles
//     for (auto & ap : angle_pubs_) {
//       const double rad = clampRad(deg2rad(double(ap.deg)), msg->angle_min, msg->angle_max);
//       int idx = int(std::lround((rad - msg->angle_min) / msg->angle_increment));
//       idx = std::max(0, std::min(int(n) - 1, idx));
//       const float v = ranges[size_t(idx)];
//       if (std::isfinite(v)) publishFloat(ap.pub, v);
//     }

//     // Optional: log lightweight line (comment if noisy)
//     // RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
//     //   "scan N=%zu valid=%zu bad=%.1f%% sigma=%.3fm spikes=%.1f%% lonely=%.1f%%",
//     //   n, valid_count, badpct, sigma, spikepct, lonelypct);
//   }

//   inline static double deg2rad(double d){ return d * M_PI / 180.0; }
//   inline static double clampRad(double x, double a, double b){ return std::min(std::max(x,a), b); }

//   static std::string vecToStr(const std::vector<int>& v) {
//     std::string s; for (size_t i=0;i<v.size();++i){ s += std::to_string(v[i]); if(i+1<v.size()) s += ","; }
//     return s;
//   }

//   void publishFloat(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& p, float v){
//     std_msgs::msg::Float32 m; m.data = v; p->publish(m);
//   }

//   // Params
//   std::string scan_topic_;
//   double spike_th_, lonely_th_;
//   std::vector<int> angles_deg_;

//   // IO
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
//   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_sigma_, pub_badpct_, pub_spikepct_, pub_lonelypct_;
//   std::vector<AnglePub> angle_pubs_;
// };

// int main(int argc, char** argv){
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ScanNoiseMonitor>());
//   rclcpp::shutdown();
//   return 0;
// }


