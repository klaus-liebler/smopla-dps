#pragma once

class Screen {
public:
	Screen();
	virtual ~Screen();
	void Update();
	void SetToForeground();
	virtual void UpdateInt()=0;
};
