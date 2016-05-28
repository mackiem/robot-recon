#pragma once
#include "qobject.h"
class SimulatorThread :
	public QObject
{
	Q_OBJECT
public:
	void init();
	SimulatorThread();
	virtual ~SimulatorThread();
};

