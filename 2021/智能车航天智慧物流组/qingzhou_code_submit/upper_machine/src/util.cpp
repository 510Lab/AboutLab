#include "util.hpp"

std::optional<double> parse_double(const std::string &str) {
	char *endp;

	double value = std::strtod(str.c_str(), &endp);

	if (endp == str.c_str()) {
		/* conversion failed completely, value is 0, */
		/* endp points to start of given string */
		return std::nullopt;
	} else if (*endp != 0) {
		/* got value, but entire string was not valid number, */
		/* endp points to first invalid character */
		return std::nullopt;
	} else {
		/* got value, entire string was numeric, */
		/* endp points to string terminating 0 */
		return value;
	}
}
