/*
 * configs.h
 *
 *  Created on: Jul 27, 2015
 *      Author: fpapinea
 */

#ifndef CONFIGS_H_
#define CONFIGS_H_

/**
 * @brief Easier Macros to use
 */
#define PRINT					Serial.print
#define PRINTLN					Serial.println

/**
 * @brief MQTT Definitions.
 */

/*!
 * Maximum MQTT packet length
 */
#define MQTT_MAX_PACKET_SIZE 		(255)

/*!
 * Publish
 */
#define MQTT_PUBLISH_DATA			("sensor/01/data/")
#define MQTT_PUBLISH_STATUS			("sensor/01/status/")

/*!
 * Subscribe
 */
#define MQTT_SUB_TOPIC_1			("global_cmd/#")
#define MQTT_SUB_TOPIC_2			("sensor_cmd/01/#")

/**
 * @brief MQTT Broker attributes
 */
#define MQTT_BROKER_ADDR			("broker.haligonia.home.com")
#define MQTT_BROKER_PORT			(1883)

/**
 * @brief MQTT Attributes
 */
#define MQTT_VERSION				(3)
#define MQTT_SENSOR_ID				("SENSOR_1")
#define MQTT_USERNAME				("use-token-auth")
#define MQTT_PASSWORD				("haligonia")
#define MQTT_KEEP_ALIVE				(10)

#endif /* CONFIGS_H_ */
