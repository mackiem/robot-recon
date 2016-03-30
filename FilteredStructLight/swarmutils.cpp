#include "swarmutils.h"

void SwarmUtils::print_vector(const std::string& name, const glm::vec3& vector) {
#ifdef DEBUG
	std::cout << name << " : " << vector.x << ", " << vector.y << ", " << vector.z << std::endl;
#endif
}
