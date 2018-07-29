#ifndef PPM_STREAM_H
#define PPM_STREAM_H

#include <ostream>

class ppm_target
{
public:
	virtual void setChannelValue(unsigned char channel, float value) = 0;
};

class ppm_file : public ppm_target
{
public:
	ppm_file(int fd);
	
public:
	void setChannelValue(unsigned char channel, float value) override;
	
private:
	int m_fd;
	
};

class ppm_wbc : public ppm_target
{
public:
	ppm_wbc(int interfacecount, char *interfaces[]);
	
public:
	void setChannelValue(unsigned char channel, float value) override;
	
private:
	
};

#endif

