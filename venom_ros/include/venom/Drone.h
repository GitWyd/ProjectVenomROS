#include <list.h>
#include <vector.h>

namespace venom{

class Drone {
public:
	Drone();
	double est_error();

protected:
	std::list<std::vector<double>> path_;
};

}	// namespace venom
