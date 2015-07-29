/*
 * service.h
 *
 *  Created on: Jul 4, 2015
 *      Author: francis-ccs
 */

#ifndef SERVICES_SERVICE_H_
#define SERVICES_SERVICE_H_

#include <status_codes.h>

class service {


	public:

		status_code_t status;

		service();
		virtual ~service();
};

/**< @brief Typedef for the class */
typedef service service_t;

#endif /* SERVICES_SERVICE_H_ */
