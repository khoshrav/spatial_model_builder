#ifndef ENTITY_H
#define ENTITY_H
#include <map>
#include <tuple>
#include <set>
#include <functional>
#include <algorithm>
#include <utility>
#include <iostream>
#include <boost/serialization/map.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>


typedef std::function<bool (std::pair<std::tuple<double, double, double>,
  long double>, std::pair<std::tuple<double, double, double>, long double>)>
  Comparator;

template<typename A, typename B>
std::pair<B,A> flip_pair(const std::pair<A,B> &p)
{
    return std::pair<B,A>(p.second, p.first);
}

template<typename A, typename B>
std::multimap<B,A> flip_map(const std::map<A,B> &src)
{
    std::multimap<B,A> dst;
    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()),
                   flip_pair<A,B>);
    return dst;
}

class Entity {
 public:
  Entity(std::string name, std::string log_dir);
  Entity();
  void observedAt(double prob,
                  double x,
                  double y,
                  double z);
  std::set<std::pair<std::tuple<double, double, double>, long double>, Comparator> find_old(void);
  std::set<std::pair<std::tuple<double, double, double>, long double>, Comparator> findProb_old(void);
  std::multimap<long double, std::tuple<double, double, double>> find();
  std::multimap<long double, std::tuple<double, double, double>> findProb();
  bool isEntity(std::string entity_type);
  void saveModel();
  void loadModel();
 private:
  std::string entity_name;
  std::string log_path;
  std::map<std::tuple<double, double, double>, long double>occupancy;
  std::map<std::tuple<double, double, double>, long double>occupancy_probs;

};

namespace boost {
  namespace serialization {

    template<uint N>
    struct Serialize
    {
        template<class Archive, typename... Args>
        static void serialize(Archive & ar, std::tuple<Args...> & t, const unsigned int version)
        {
            ar & std::get<N-1>(t);
            Serialize<N-1>::serialize(ar, t, version);
        }
    };

    template<>
    struct Serialize<0>
    {
        template<class Archive, typename... Args>
        static void serialize(Archive & ar, std::tuple<Args...> & t, const unsigned int version)
        {
            (void) ar;
            (void) t;
            (void) version;
        }
    };

    template<class Archive, typename... Args>
    void serialize(Archive & ar, std::tuple<Args...> & t, const unsigned int version)
    {
        Serialize<sizeof...(Args)>::serialize(ar, t, version);
    }

  }
}
#endif
