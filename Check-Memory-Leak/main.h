#ifndef MAIN_H_
#define MAIN_H_

#define VREFINT             (1.21)
#define ADC_RESOLUTION      (4095.0)
#define RATIO               (1.3)
#define VCC					(3.3)

typedef struct sx1278_node
{
	int node_id;
	int gate_id;
	char battery[10];
	int period;
	char threshold[10];
} sx1278_node_t;

#endif 