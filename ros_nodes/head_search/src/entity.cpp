#include "head_search/entity.h"

//Comparator to order map
Comparator compFunctor =
  [](std::pair<std::tuple<double, double, double>, long double>elem1,
    std::pair<std::tuple<double, double, double>, long double>elem2){
    return elem1.second >= elem2.second;
  };

// Entity constructor
Entity::Entity(std::string name, std::string log_dir) : entity_name(name), log_path(log_dir)
{}

Entity::Entity() {
  saveModel();
}

// Load model from log_path
void Entity::loadModel()
{
  printf("Loading file\n");
  std::string path_dir = log_path+entity_name;
  std::ifstream fmodel_file;
  fmodel_file.open(path_dir+"/frequency.map");
  boost::archive::text_iarchive farch(fmodel_file);
  farch >> occupancy;
  fmodel_file.close();
  std::ifstream pmodel_file;
  pmodel_file.open(path_dir+"/probability.map");
  boost::archive::text_iarchive parch(pmodel_file);
  parch >> occupancy_probs;
  pmodel_file.close();
  printf("Loaded %d=%d for %s\n", occupancy.size(), occupancy_probs.size(), entity_name.c_str());
}

// Dump models to log_path
void Entity::saveModel()
{
  std::string path_dir = log_path+entity_name;
  boost::filesystem::path dir(path_dir);

  if(!(boost::filesystem::exists(dir))){
      printf("%s Doesn't Exists\n", path_dir.c_str());

      if (boost::filesystem::create_directory(dir))
          printf("....Successfully Created !\n");
  }
  std::ofstream fmodel_file;
  fmodel_file.open(path_dir+"/frequency.map");
  boost::archive::text_oarchive farch(fmodel_file);
  farch << occupancy;
  fmodel_file.close();
  std::ofstream pmodel_file;
  pmodel_file.open(path_dir+"/probability.map");
  boost::archive::text_oarchive parch(pmodel_file);
  parch << occupancy_probs;
  pmodel_file.close();
}


bool Entity::isEntity(std::string entity_type) {
  return (entity_name.compare(entity_type) == 0);
}

// Add position information for entity
void Entity::observedAt(double prob, double x, double y, double z) {
  std::tuple<double, double, double> position(x, y, z);
  occupancy_probs[position] = static_cast<long double>((occupancy_probs[position]*occupancy[position]) + prob)/
          static_cast<long double>(occupancy[position]+1);
  occupancy[position]++;
}

// Return the set of all positions sorted by value
std::set<std::pair<std::tuple<double, double, double>,long double>, Comparator> Entity::find_old() {
  std::set<std::pair<std::tuple<double, double, double>,long double>,
  Comparator> setOfPositions(
  occupancy.begin(), occupancy.end(), compFunctor);
  // std::set<std::pair<std::tuple<double, double, double>, double>> positions;
  // for (std::pair<std::tuple<double, double, double>, double> element : setOfPositions) {
  //   positions.insert(element);
  // }
  return setOfPositions;
}

// Return the set of all positions sorted by probabilities
std::set<std::pair<std::tuple<double, double, double>, long double>, Comparator> Entity::findProb_old() {
  std::set<std::pair<std::tuple<double, double, double>, long double>,
  Comparator> setOfPositions(
  occupancy_probs.begin(), occupancy_probs.end(), compFunctor);
  return setOfPositions;
}

std::multimap<long double, std::tuple<double, double, double>> Entity::find()
{
  std::multimap<long double, std::tuple<double, double, double>> dst = flip_map(occupancy);
  return dst;
}

std::multimap<long double, std::tuple<double, double, double>> Entity::findProb()
{
  std::multimap<long double, std::tuple<double, double, double>> dst = flip_map(occupancy_probs);
  return dst;
}
