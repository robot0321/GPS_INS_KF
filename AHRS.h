#include <vector>

class AHRS{
private:
	
	std::vector<float> w_1; //platform(initially measured frame)
	std::vector<float> Q_2;
	std::vector<float> w_2;
	std::vector<float> transforming_buffer;

public:
	std::vector<float> Q_1;
	std::vector<float> Q_0;
	std::vector<float> w_0; //body frame

	AHRS();
	AHRS(std::vector<float> init_Q);
	~AHRS();

	void attitude_update(std::vector<float> w, float dt);
	std::vector<float> frame_transformer(std::vector<float> Q, std::vector<float> v);
};