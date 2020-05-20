#pragma once
#include <gram_savitzky_golay/spatial_filters.h>
#include <SpaceVecAlg/SpaceVecAlg>

template <typename T>
struct SpaceVecAlgFilter : public gram_sg::EigenVectorFilter<decltype(T().vector())>
{
  using Filter = gram_sg::EigenVectorFilter<T>;
 public:
  SpaceVecAlgFilter();
  void reset();
  void reset(const T & f)
  {
    Filter::reset(f.vector());
  }
  void add(const T & f)
  {
    Filter::add(f.vector());
  }
  T filter()
  {
    return Filter::filter();
  }
};
