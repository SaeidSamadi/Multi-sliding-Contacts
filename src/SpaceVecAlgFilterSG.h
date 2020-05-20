#pragma once
#include <gram_savitzky_golay/spatial_filters.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

template <typename T>
struct SpaceVecAlgFilterSG : public gram_sg::EigenVectorFilter<decltype(T().vector())>
{
  using VectorT = decltype(T().vector());
  using Filter = gram_sg::EigenVectorFilter<VectorT>;

 public:
  SpaceVecAlgFilterSG()
  {
  }

  void reset(const mc_rtc::Configuration & config, double dt, const T& f)
  {
    int m = config("m");
    int t = config("t");
    int n = config("n");
    int s = config("s");
    LOG_INFO("m: " << m);
    LOG_INFO("t: " << t);
    LOG_INFO("n: " << n);
    LOG_INFO("s: " << s);
    this->reset(gram_sg::SavitzkyGolayFilterConfig(m,t,n,s,dt), f);
  }
  void reset(const gram_sg::SavitzkyGolayFilterConfig & config, const T f)
  {
    Filter::reset(config, f.vector());
  }
  void reset(const T & f)
  {
    Filter::reset(f.vector());
  }
  void add(const T f)
  {
    LOG_INFO("add " << f.vector().transpose());
    Filter::add(f.vector());
  }
  T filter() const
  {
    T res;
    res.vector() = Filter::filter();
    LOG_INFO("res " << res.vector().transpose());
    return res;
  }
};
