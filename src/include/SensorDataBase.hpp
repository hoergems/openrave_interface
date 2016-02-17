#ifndef _SENSOR_DATA_BASE_HPP_
#define _SENSOR_DATA_BASE_HPP_
#include <openrave-core.h>

namespace shared {

class SensorDataBase {
	
};

template<typename SensorDataType>
class SensorData: public SensorDataBase {
public:
	SensorData(OpenRAVE::SensorBase::SensorDataPtr &sensor_data):
	    sensor_data_(boost::static_pointer_cast<SensorDataType>(sensor_data)){
		
	}	
	
	boost::shared_ptr<SensorDataType> getSensorData() {
		return sensor_data_;
	}
	
protected:
    boost::shared_ptr<SensorDataType> sensor_data_;
};

}

#endif