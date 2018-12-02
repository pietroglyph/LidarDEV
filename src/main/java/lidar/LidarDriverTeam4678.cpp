// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.



#include <iostream>
#include <fstream>
#include "Lidar.h"
#include "../Robot.h"
#include "SerialPort.h"
#include "../RobotMap.h"
#include "Autonomous/RobotHelpers.h"
#include <ctime>
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Lidar::Lidar() : frc::Subsystem("Lidar") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONSt
	printf("Lidar Initialization!\n");
	//SerialPort serport = frc::SerialPort(115200);
	serport = new frc::SerialPort(115200,frc::SerialPort::Port::kUSB,8,frc::SerialPort::Parity::kParity_None,frc::SerialPort::StopBits::kStopBits_One); // Set up serial port.
	serport->SetWriteBufferMode(frc::SerialPort::kFlushOnAccess);
	serport->SetFlowControl(frc::SerialPort::FlowControl::kFlowControl_None);
	serport->SetWriteBufferSize(128); // Don't need a lot of space for writing.
	serport->SetReadBufferSize(1024); // allow up to 1024 characters to be stored for reading.
	serport->Flush(); // Clear out any old data.
	samplecount = 0;
	cstate = 0;
	txseq = 0;
	prevangle = -1; // prevangle is not valid yet.
	tocnt = 0; // Time out counter.
	chcnt = 100;
	glob_lidar_ready = 0;
	cubeFindCase = 0;
	cubeSquaringCase = 0;
	cubeIntakeCase = 0;
	glob_lidar_may_run = 0;
	doneGo = false;
	prevtstamp = (int)(Timer::GetFPGATimestamp()*1000000); // get value in microseconds.

}

void Lidar::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Lidar::Periodic() {
	// Put code here to be run every loop
	int x,ch;
	time_t t=time(0);
	struct tm *now;

	if (logfile.is_open())
		{
		sprintf(buf,"E,%d,%d,%f,%f\n",Robot::driveTrain->getLeftEncoder()->Get(),Robot::driveTrain->getRightEncoder()->Get(),Robot::ahrs->GetAngle(),Timer::GetFPGATimestamp());
		logfile.write(buf,strlen(buf));
		}
	if (txcmd != 0)
		{
		switch(txcmd)
			{
			case 1: // transmit data to get information packet from lidar
				//printf("Sending Info request string to serial port\n");
				serport->Write("\xA5\x5A\x14\x00\x00\x00\x04",7);
				//printf("Write(\\xA5\\x5A\\x14\\x00\\x00\\x00\\x04)\n");
				//printf("Get Info Requested A5 5A 14 00 00 00 04\n");
				txcmd = 0; // indicate this request has been sent.
				cstate=0;
				break;
			case 2: // stop - send to halt data being transmitted from lidar
				serport->Write("\xA5\x25",2);
				//printf("Stop A5 25 sent.\n");
				txcmd = 0;
				cstate = 0;
				break;
			case 3: // Reset - perform a reboot of the lidar system
				// Make sure we wait 2ms (1/500 sec) before sending another command.
				// should be no problem given that we'll only transmit at 50hz
				serport->Write("\xA5\x40",2);
				//printf("RESET A5 40");
				txcmd = 0;
				cstate = 0;
				break;
			case 4: // Express Scan.  There are other options but we're only implementing this one.
				serport->Write("\xA5\x82\x05\x00\x00\x00\x00\x00\x22",9);
				//printf("Express Scan Initiated\n");
				txcmd = 0;
				cstate = 0;
				break;
			case 5: // With USB interface board, we should be able to control motor speed 0 to 1023, default 660
				// Will need checksum calculation for this,  Start with 0 and ^= all bytes, including 0xA5 Sync
				//printf("Sending motor PWM command 0xF0\n");
				//serport->Write("\xA5\xF0\x02\x94\x02\xC1",6); // See if this will start the motor.
				//serport->Write("\xA5\xFF\x04\x00\x00\x00\x80\xDE",8); // Get flag to see if board supports PWM command.
				//serport->Write("\xA5\xFF",2);
				serport->Write("\xA5\xF0\x02",3);
				chksum = 0xA5^0xF0^0x02;
				serport->Write((char *)&pwmspeed,2);
				chksum ^= (pwmspeed & 0xFF);
				chksum ^= ((pwmspeed >> 8) & 0xFF); // calculate checksum.
				serport->Write((char *)&chksum,1);
				//printf("Write(\\A5\\F0\\02\\%02X\\%02X\\%02X)\n",pwmspeed&0xFF,pwmspeed>>8,chksum);
				txcmd = 0;
				break;
			case 6: // Get Health
				//printf("Sending Get Health 0x52\n",cstate);
				serport->Write("\xA5\x52",2);
				txcmd = 0;
				cstate = 0;
				break;
			case 7: // Send A5 50
				serport->Write("\xA5\x50");
				//printf("Write(\\xA5\\x50)");
				txcmd = 0;
				cstate = 0;
				break;
			case 8: // Send A5 59
				serport->Write("\xA5\x59");
				//printf("Write(\\xA5\\x59)");
				txcmd = 0;
				cstate = 0;
				break;
			}
		}
	loopcount = 0; // serport->Read() does not always read all available bytes.  Sometimes, we only get a few
	// loopcount will allow this routine to loop up to 25 times in order to try to get all the bytes
	bytes_available = serport->GetBytesReceived();
	tim1 = Timer::GetFPGATimestamp(); // See how much time this takes.
	while((loopcount < 25)&&(bytes_available > 0))
		{
		loopcount++;
		if (bytes_available > 1024)
			bytes_to_read = 1024;
		else
			bytes_to_read = bytes_available;
		if (bytes_to_read > 0)
			{
			 //printf("bytes_to_read=%d\n",bytes_to_read);
			 rdcnt = serport->Read(rxbuf,bytes_to_read);
			 for(x = 0;x<rdcnt;x++) // process each character in the serial port buffer.
				{
				ch = *(rxbuf+x); // get the character
				// printf("ch=%02x, cstate=%d, dcnt=%d\n",ch,cstate,dcnt);
				//if (cstate < 4)
				//if (chcnt < 16)
				//	{
				//	printf("%02X ",ch);
				//	chcnt++;
				//	}
				switch(cstate)
					{
					case 0: // expecting a packet start character
						if (ch == 0xA5)
							cstate++; // go to next state if we get a valid packet start character
						break;
					case 1: // Expecting char 2 of the response descriptor
						if (ch == 0x5A)
							{
							cstate++; // valid character, advance to next state
							bcnt = 0; // need to count 4 bytes in next case
							rdat = 0; // place result in rdat
							}
						else
							cstate=0; // not what we expected, back to state 0
						break;
					case 2: // 30 bits of response length.  Get all 32 and then split it.
						rdat |= (((unsigned int)(ch)) << (bcnt * 8)); // get data, LSB first.
						bcnt++;
						if (bcnt == 4)
							{
							sendmode = rdat >> 30; // get the 2 bits of send mode. 0=single response, 1=multiple response.
							nbytes = rdat & 0x3FFFFFFF; // strip to upper 2 bit from rdat and that's our data length.
							printf("\nExpecting %d data bytes\n",nbytes);
							if (nbytes < 100)
								cstate++;
							else
								cstate = 0; // error in length, back to state 0
							}
						break;
					case 3: // expecting data type
						dtype = ch; // 0x82 for Express Packet, 0x04 for response to get_info
						dcnt = 0; // prepare to count payload data bytes (counts to nbytes)
						cstate++;
						break;
					case 4: // expecting payload data of nbytes (can be up to 84 bytes for express scan mode)
						// will be considerably less when we get the info data block.
						if (dcnt < 128)
							payload[dcnt] = ch; // store the data as long as it doesn't exceed the buffer
						dcnt++;
						if (dcnt >= nbytes)
							{
							// payload is complete.  process the data
							//printf("Got a complete payload, dtype=%d bytes_to_read=%d, actually read=%d, available=%d\n",dtype,bytes_to_read,rdcnt,bytes_available);
							chcnt=0; // Show any characters we get on rs232 port, starting now
							if (dtype == 0x18) // This is info, get some of it
								{
								model = payload[0];
								firmware1 = payload[1];
								firmware2 = payload[2];
								hardware = payload[3];
								serial = (payload[4])+((unsigned int)payload[5]<<8)+((unsigned int)payload[5]<<16)+((unsigned int)payload[5]<<24);
								// we'll ignore the other serial numbers for now.
								printf("Lidar model:%d\n",model);
								printf("Lidar firmware %d.%d\n",firmware2,firmware1);
								printf("Lidar hardware %d\n",hardware);
								printf("Lidar serial number %d\n",serial);
								//txcmd = 6; // go check health
								cstate = 0;
								}
							else if (dtype == 0x06)
								{
								printf("Health = %02X\n",payload[0]);
								printf("Error Code=%02X%02X\n",payload[2],payload[1]);
								//txcmd = 5; // Try to start motor now.
								cstate = 0;
								}
							else if (dtype == 0x82) // This is the express scan data.  Time to sort out all the data bits and pieces
								{
								// First 4 bytes - sync1, 2, checksum, start angle, S indicator.
								synchbyte = (payload[0] & 0xF0) + ((payload[1] >> 4) & 0x0F); // This must be 0xA5
								if (synchbyte == 0xA5)
									{
									stangle = payload[2] + ((payload[3] & 0x7F) << 8); // actual start angle value.

									chksum = (payload[0] & 0x0F) + ((payload[1] << 4) & 0xF0); // 8 bit checksum
									//printf("stangle=%6.2f, chksum=%02X\n",(double)stangle/64.0,chksum);

									//int xx = 0;
									//for(acnt = 0;acnt < 0x82;acnt++)
									//	{
									//	printf("%02X ",payload[acnt]);
									//	xx ^= payload[acnt];
									//	}
									//printf("\nxx=%02X\nstangle=%d,prevangle=%d\n",xx & 0xFF,stangle,prevangle);

									tstamp = (int)(Timer::GetFPGATimestamp()*1000000); // get value in microseconds.
									// Don't worry about FPGATimestamp roll over.  It's 72 minutes
									// Possible mess up if running robot for a long time during program testing.

									// if we have a valid prevangle, update the cabdat[] from last express data set
									// with the correct angles and then add the cabdat[] to the full scan array and
									// then we can re-use cabdat[] to store data from this payload.
									if (prevangle != -1)
										{
										for(acnt=0;acnt<32;acnt++) // Do all 32 angles from last set of cabins
											{ // place actual angle values in previous cabin[] array.
											if (stangle >= prevangle)
												cabdat[acnt].angle = prevangle + (stangle - prevangle) * acnt / 32 + cabdat[acnt].angle;
											else
												cabdat[acnt].angle = prevangle + (23040 + stangle - prevangle) * acnt / 32 + cabdat[acnt].angle;
											//if (cabdat[acnt].angle >= 23040)
											//	cabdat[acnt].angle -= 23040; // Correct for angles above 360 deg
											tmpdat[samplecount].dist = cabdat[acnt].dist;
											tmpdat[samplecount].angle = cabdat[acnt].angle;
											tmpdat[samplecount].tstamp = prevtstamp + (tstamp - prevtstamp) * acnt / 32; // record correct time-stamp values.
											if (samplecount < 1023)
												samplecount++;
											}
										}

									prevtstamp = tstamp;
									//printf("\nstangle=%d,%10.2f\n",stangle,(double)stangle/64.0);

									sss = payload[3] >> 7; // This is indicator of new scan (1 for first data set of a scan.)
									// if sss is 1, that means we have compiled a complete array of readings.
									// this is a good time to copy them to the array to be used by the main lidar
									// processing routines and then set the indicator to tell the forground
									// that data is ready.
									if ((sss == 1)||(stangle < prevangle)) // sss doesn't seem to work, just use cross over in angle value.
										{
										for(acnt = 0;acnt < samplecount;acnt++) // copy entire array.
											{
											lidat[acnt].angle = tmpdat[acnt].angle;
											lidat[acnt].dist = tmpdat[acnt].dist;
											lidat[acnt].tstamp = tmpdat[acnt].tstamp;
											//printf("%d,%d,%d\n",lidat[acnt].angle,lidat[acnt].dist,lidat[acnt].tstamp);
											if ((lidat[acnt].angle > 90*64)&&(lidat[acnt].angle < 270*64)&&(lidat[acnt].dist != 0))
												{
												if (logfile.is_open())
													{
													sprintf(buf,"D,%d,%d,%d\n",lidat[acnt].angle,lidat[acnt].dist,lidat[acnt].tstamp);
													logfile.write(buf,strlen(buf));
													}

												}
											}
										glob_lidar_ready = 1; // main processing routine can set this to 0 so
										// it can detect when the next valid lidar scan is ready for processing.
										glob_lidar_count = samplecount;
										samplecount = 0; // begin storing new data.
										skcnt=0; // Keep rebooter sleeping, data is coming in.
										}

									prevangle = stangle; // update prevangle to equal this angle for next data set.

									for(cabins = 0;cabins < 16;cabins++)
										{ // process the 16 cabins
										caboff = 4+cabins * 5; // offset to data start for this cabin
										dist1 = (payload[caboff] >> 2) + ((int)payload[caboff+1] << 6); // distance value
										// Documentation on angle data is not correct.  [caboff] may be msbit, may be lsbits.  Not sure.
										// going with the assumption that they're bits 4 and 5 and that [caboff + 4] is bits 0-3
										// Note that these are signed 5 bit values. 0b11111 = -1, 0x10000 = -16.
										// May need to have a look at the value to make sure we have the bits figured out correctly.
										//
										delta1 = ((payload[caboff+1] & 0x03) << 4) + (payload[caboff+4] & 0x0F); // 5 bits of angle delta
										if (delta1 >= 16)
											delta1-=32; // these are negative values 0b11111 (31) is actually -1.
										dist2 = (payload[caboff + 2] >> 2) + ((int)payload[caboff+3] << 6); // distance value
										delta2 = ((payload[caboff+3] & 0x03) << 4) + ((payload[caboff+4] & 0xF0) >> 4); // 5 bits of angle delta
										if (delta2 >= 16)
											delta2-=32;
										// have distance and delta angle.  These delta's are corrections to the
										// angle that would be calculated using direct interpolation between the stangle of this
										// set of 32 readings and the stangle of the next 32 readings. By doing (stangle[1] - stangle[2]) / 32
										// The trick is we need stangle[2] before we can calculate the angles for data of stangle[1]
										// with confident accurace.  Since angles between 270 and 90 won't matter to us, this is not a big deal.
										// When preparing the data set, it will be necessary to go back and update the previous 32
										// angles based on the stangle value of the new packet.
										cabdat[cabins<<1].dist = dist1;
										cabdat[cabins<<1].angle = delta1; // storing delta1 for now, will update with actual angle next reading.
										cabdat[(cabins<<1)+1].dist = dist2;
										cabdat[(cabins<<1)+1].angle = delta2;
										}
									dcnt = 0; // expect another 0x82 bytes, stay in this state.
									}
								else // Synch byte failed to be 0xA5.  Data is no longer aligned.
									{ // Stop the transfer and then re-initated it
									txcmd = 2; // This will send the scan stop request.
									txseq = 5; // Go to state 5 which will stop current scan and start another one.
									}
								}
							// cstate = 0; // Back to cstate 0 when we've received all the bytes we expected.
							} // if(dcnt >= nbytes)
						break;
					} // switch(cstate)
				} // (for x < bytes_to_read)
			} // if there were bytes to read.
		 // if ((bytes_to_read == 0)&&(txcmd == 0)) // If there's nothing to read, consider a sequence of commands to try to get scanner going.
		bytes_available = serport->GetBytesReceived();
		}
	if (txcmd == 0)
		 { // We really don't need to do a lot.  Just tell it to go, now that we have the speed thing figured out.
		 // We can modify this part to start the scan and monitor for problems.  If we stop getting data, this part of the code
		 // can reset the system and re-start.
		 //printf("txseq=%d\n",txseq);
		 switch(txseq)
			 {
			 case 0:// Start up the motor as the first thing we do.
				 if (glob_lidar_may_run != 0) // Wait for signal from main program loop to tell lidar to run.
					 {
					 txcmd = 5; // start up the motor
					 pwmspeed = 660;
					 // txcmd = 0;
					 skcnt = 0; // use this to delay for a 100 count (2 seconds) to allow lidar to spin up.
					 if (!logfile.is_open())
						 {
						 now = localtime(&t);
						 sprintf(buf,"/media/sda1/LL%02d%02d%02d%02d%02d%02d.txt",now->tm_year,now->tm_mon,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
						 logfile.open(buf,std::ios::out | std::ios::binary);
						 }
					 else
					 	 {
						 now = localtime(&t);
						 sprintf(buf,"/media/sdb1/LL%02d%02d%02d%02d%02d%02d.txt",now->tm_year,now->tm_mon,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
						 logfile.open(buf,std::ios::out | std::ios::binary);
					 	 }
					 txseq++;
					 }
				 break;
			 case 1: // after the 2 second delay, start the scan and look for data.
				 skcnt++;
				 if (skcnt >= 100)
					 {
					 serport->Flush(); // Clear data from serial port.
					 cstate = 0; // begin at state 0 to obtain valid packets.
					 txcmd = 1;
					 txseq++;
					 }
				 break;
			 case 2: // Redundant motor start up.
				 txcmd = 5; // start up the motor
				 pwmspeed = 660;
				 txseq++;
				 break;
			 case 3:
				 txcmd = 4; // Begin scanning in express mode.
				 skcnt = 0; // next state will increase this.  It gets cleared when we receive valid scan data.
				 txseq++;
				 break;
			 case 4:
				 skcnt++;  // if this gets to 200, it's been 4 seconds since we got data. Restart the scanner.
				 if (skcnt >= 500)
					 txseq = 5; // restart.
				 if (glob_lidar_may_run == 0)
					 txseq = 7; // stop everything, lidar may shut down.
				 //tocnt++;
				 //if (tocnt >= 50)
				 //	 glob_lidar_may_run = 0;
				 break;
			 case 5:
				 txcmd = 2; // Send a stop to stop the express scanning.
				 printf("\nLidar scan restarted!!\n");
				 txseq++;
				 break;
			 case 6:
				 txseq=3; // Back to txseq=3 which will start another express scan.
				 break;
			 case 7:
				 txcmd = 2;
				 txseq++;
				 break;
			 case 8:
				 pwmspeed = 0; // stop the motor.
				 txcmd = 5; // Motor speed to 0
				 txseq++;
				 break;
			 case 9:
				 if (glob_lidar_may_run == 0)
				 	 {
					 if (logfile.is_open())
						 logfile.close();
					 txseq = 0; // back to state 0 once lidar has been shut down.
					 // Intialize all the variables that will allow another lidar sequence to begin.
					 samplecount = 0;
					 cstate = 0;
					 txseq = 0;
					 prevangle = -1; // prevangle is not valid yet.
					 tocnt = 0; // Time out counter
					 }
				 break;
			 }
		 }
	tim2 = Timer::GetFPGATimestamp(); // See how much time this takes.
	//printf("Time in Lidar::Periodic = %12.9f, loopcount=%d\n",tim2 - tim1,loopcount);
	} // proc_serial()

void Lidar::startLidar()
	{
	glob_lidar_may_run = 1;
	}

void Lidar::stopLidar()
	{
	glob_lidar_may_run = 0;
	}

void Lidar::readLidar()
	{
	glob_lidar_ready = 0;
	}

bool Lidar::readComplete()
	{
	return glob_lidar_ready;
	}

void Lidar::convertToXY()
	{
	int j = 0;
	for(int i=0;i<glob_lidar_count;i++)
		{
		if(!lidat[i].dist)
			continue;
		double rad = M_PI * ((double)lidat[i].angle / 64.0) / 180;
		lidatXY[j].x = ((((double)lidat[i].dist) * std::sin(rad)));
		lidatXY[j].y = -((((double)lidat[i].dist) * std::cos(rad)));
		//printf(" %i,%i",lidatXY[j].x,lidatXY[j].y);
		j++;
		}
	printf("\n");
	xyCount = j;
	}

void Lidar::filterData(bool convertXY, double leftLimit, double rightLimit, double minDistance, double maxDistance)
	{
	unsigned int startidx = 0, n=0;
	double dist, angle;
	//TODO: remove this line
	//convertToXY();
	for(int i = 0;i<glob_lidar_count;i++)
		{
		angle = ((double)lidat[i].angle / 64.0);
		//Angle
		if(angle < (180-leftLimit))
			continue;
		if(angle > (180+rightLimit))
			continue;
		//Distance
		dist = lidat[i].dist;
		if((dist < minDistance)||(dist > maxDistance))
			continue;

		lidFiltered[n] = lidat[i];
		if(convertXY)
			{
			double rad = M_PI * (lidat[i].angle / 64.0) / 180;
			lidatXY[n].x = (std::round((lidat[i].dist) * std::sin(rad)));
			lidatXY[n].y = -(std::round((lidat[i].dist) * std::cos(rad)));
			}
		n++;
		}

	if(convertXY)
		xyCount = n;
	filteredCount = n;

	//TODO: Delete this debug code:
	//printf("Cont: %i\n",xyCount);
	//for(int i = 0; i<xyCount; i++)
	//	printf(" %i,%i",lidatXY[i].x,lidatXY[i].y);
	//printf("\n");
	}

/*
* Line finding algorithm
* 	The data is sorted based on their direction from (0,0), the first point
* 		for example is close to 90 degrees from (0,0) the last point is close
* 		to -270.
* 		The data must also be in (x,y) coord
* 	The following loop go through this data calculating the angles from one
* 		point to the next and determines if the two points are in a line.
* 		This is a very basic discription
*	Let me walk you through it a little
* 		Start at point 0:
* 			The code creates a new line and sets the start and end to point 0
* 			Then goes to the next point
* 		Point 1:
* 			Since there is no angle to compare to yet as the current line
* 				only exists on a single point
* 			The distance between this point and the line is checked
* 				If the distance is too large then the current line is
* 					set to point 1 and then goes to the next point
* 				If the distance is good then the end point of the current
* 					line is set to point 1 and the angle of the line is
* 					calculated. Then goes to the next point
* 		Point 2:
* 			Assuming that there is a valid angle for the line now
* 				The angle from the line's end point and this point is calculated
* 				Then compared to the angle of the line
* 				If the angle is out of range then the code checks if point 2 is
* 					an outlier by checking the next few points and comparing
* 					their angles to the current line
* 					If not an outlier, then a new line is created at point 2
* 						This new line becomes the current one and then the code
* 							goes to the next point
* 				If the angle is within the range then the end point of the current
* 					line is set to point 2 and the angle is recalculated
* 					Then the code goes to the next point
* 		This continues for all the data points
*/
void Lidar::FindLines(){
	//Find lines
	double distX, distY, totalDist, angle, angle2, diff;
	bool newline; //used for determine outliers and whether or not to create a new line
	int n,m,NumLines=0;
	int skipto = 0;
	//Loop through the data
	for(n=0;n<xyCount;n++)
		{
		//Allows the code to skip over data points that are determine to
		//	be outliers
		if(skipto != 0)
			{
			if(n < skipto)
				continue; //Skip this point until n == skipto
			skipto = 0;//reset the skip value
			}
		//To start thing the code sets the first line to start and end
		//	at point 0
		if(n == 0)
			{
			//set start position
			lines[0].start.x = lidatXY[n].x;
			lines[0].start.y = lidatXY[n].y;
			//set end position
			lines[0].end.x = lidatXY[n].x;
			lines[0].end.y = lidatXY[n].y;
			//Set current line to 0
			NumLines = 0;
			continue; //Go to next point
			}

		//Calculate the differences between this point and the end of the
		//	current line
		distX = (lidatXY[n].x - lines[NumLines].end.x);
		distY = (lidatXY[n].y - lines[NumLines].end.y);
		//Total distance between this point and the end of the current line
		totalDist = sqrt(pow(distX,2)+pow(distY,2));

		//Angle calculation using inverse sin of the rise divided by the hypotenuse
		//	Then its converted from radians into degrees
		angle = (180 * (asin(distY / totalDist))) / M_PI;

		//Checks if there is a valid angle to compare
		//	If both start and end points are the same then there is no valid angle
		//	and the code must add it
		if((lines[NumLines].start.x == lines[NumLines].end.x)&&(lines[NumLines].start.y == lines[NumLines].end.y))
			{
			//Check if this point is within range
			//	If it is too far away then reset this line and continue
			if(totalDist > MAXDISTNEWLINE)
				{
				//Set start position
				lines[NumLines].start.x = lidatXY[n].x;
				lines[NumLines].start.y = lidatXY[n].y;
				//Set end position
				lines[NumLines].end.x = lidatXY[n].x;
				lines[NumLines].end.y = lidatXY[n].y;
				continue;
				}
			//This point is in range, set the end point equal to it
			lines[NumLines].end.x = lidatXY[n].x;
			lines[NumLines].end.y = lidatXY[n].y;
			//Set the angle between this point and the line's end point
			//	as the line's new direction
			lines[NumLines].length = totalDist;
			lines[NumLines].angle = angle;
			continue;//Go to the next point
			}

		//Check if this point is suitable as a new end point of the current line
		//	The code checks of the angle and distance are with the range defined
		//	by the constants at the top.
		//		If angle is greater than the current line angle plus high range
		//		If angle is less than the current line angle minus low range
		//		if distance is greater than the max distance
		//			Then check if this is an outlier or the beginning of a new line
		if(((angle > (lines[NumLines].angle + MAXANGLERANGE))||(angle < (lines[NumLines].angle - MINANGLERANGE)))||(totalDist > MAXDISTRANGE))
			{
			//New line check: assume true
			newline = true;
			//To determine if this point is an outlier the next few points
			//	must be checked, if one of them is in line with the current
			//	line then this current point must be and outlier
			for(m=n+1;m<(OUTLIERCHECK + n + 1);m++)
				{
				if(m > xyCount)
					{
					newline = false;
					break;
					}
				//Calculate the differences between this point and the end of the
				//	current line
				distX = (lidatXY[m].x - lines[NumLines].end.x);
				distY = (lidatXY[m].y - lines[NumLines].end.y);
				totalDist = sqrt(pow(distX,2)+pow(distY,2));
				//Angle calculation
				angle2 = (180 * (asin(distY / totalDist))) / M_PI;

				//Check if this point is in range
				if(((angle2 < (lines[NumLines].angle + MAXANGLERANGE))&&(angle2 > (lines[NumLines].angle - MINANGLERANGE)))&&(totalDist < MAXDISTRANGE))
					{
					//At this point the point is in range
					newline = false;//Do not create a new line, continue with current one
					skipto = m;//Skip the outliers
					break;//Exit this for loop back into the main one
					}
				}
			//Check if a new line needs to be made
			if (newline)
				{
				//increment the line count
				NumLines++;
				//Check that the limit hasn't been reached
				if (NumLines == 100)
					break;
				//Set start position for the new line
				lines[NumLines].start.x = lidatXY[n].x;
				lines[NumLines].start.y = lidatXY[n].y;
				//Set end position for the new line
				lines[NumLines].end.x = lidatXY[n].x;
				lines[NumLines].end.y = lidatXY[n].y;
				}
			continue;//Go to next point
			}

		//If this code is reached then this point must be in range
		//	set it as the new end point for the current line
		lines[NumLines].end.x = lidatXY[n].x;
		lines[NumLines].end.y = lidatXY[n].y;

		//The actual direction of this line changes and must be recalculated
		distX = (lines[NumLines].end.x - lines[NumLines].start.x);
		distY = (lines[NumLines].end.y - lines[NumLines].start.y);
		totalDist = sqrt(pow(distX,2)+pow(distY,2));

		lines[NumLines].length = totalDist;
		lines[NumLines].angle = (180 * (asin(distY / totalDist))) / M_PI;
		}
	//NumLines++;
	linecnt = NumLines;

	//TODO: Remove this debug stuff!
	for(int i = 0; i<(linecnt+1); i++){
		//printf("Line %i: start(%i, %i) end(%i, %i) angle=%f length=%i \n",i, lines[i].start.x,lines[i].start.y, lines[i].end.x,lines[i].end.y,lines[i].angle,lines[i].length);
		if(logfile.is_open())
		{
			sprintf(buf,"L,%i,%i,%i,%i,%f,%i\n",lines[i].start.x,lines[i].start.y, lines[i].end.x,lines[i].end.y,lines[i].angle,lines[i].length);
			logfile.write(buf,strlen(buf));
		}
	}


}

void Lidar::checkLinesForCubes(double frangle, double toangle){
	int n = 0, prev = 0;
	double diffWidth, diffHieght, angle, dist, distX, distY;
	tpPoint center;
	for(int i = 0; i<linecnt; i++){
		if(prev == 1){
			//Check how far this line is from the prev cube
			distX = (lines[i].start.x - lines[i-1].end.x);
			distY = (lines[i].start.y - lines[i-1].end.y);
			dist = sqrt(pow(distX,2)+pow(distY,2));
			//If too close skip it
			if (dist < MAXDISTRANGE){
				prev = 0;
				continue;
			}
		}
		diffWidth = std::abs(CUBEWIDTH - lines[i].length);
		diffHieght = std::abs(CUBEHIEGHT - lines[i].length);
		if (((diffWidth < CUBERANGEWIDTH)||(diffHieght < CUBERANGEHIEGHT))&&(lines[i].angle > frangle)&&(lines[i].angle < toangle)){
			cubes[n].location.x = (lines[i].start.x + lines[i].end.x) / 2;
			cubes[n].location.y = (lines[i].start.y + lines[i].end.y) / 2;
			//Distance from 0,0 (lidar)
			cubes[n].distance = sqrt(pow(cubes[n].location.x,2) + pow(cubes[n].location.y,2));
			//Angle from the front from 0,0 (lidar)
			cubes[n].angle = (180 * (asin((double)(cubes[n].location.x) / (double)(cubes[n].distance)))) / M_PI;
			// prev = 1; // Commented out by Maurice 18-Mar-2018.  Caused process to miss first cube.
			n++;
		}
	}
	cubecnt = n;
	for(int i = 0; i<cubecnt; i++)
		{
		if(logfile.is_open())
			{
			sprintf(buf,"C,%i,%i,%i,%f\n",cubes[i].location.x,cubes[i].location.y, cubes[i].distance,cubes[i].angle);
			logfile.write(buf,strlen(buf));
			}
		//printf("Cube %i: Loca(%i, %i) dist=%i angle=%f\n",i, cubes[i].location.x,cubes[i].location.y, cubes[i].distance,cubes[i].angle);
		}
}

void Lidar::calculatePathToNearestCube()
	{
	double angle2use;
	if (cubecnt == 0) {
		rightcm = 0;
		leftcm = 0;
		doneGo = true;
		return;
	}
	printf("Cubes: %i\n",cubecnt);
	unsigned int idx = 0;
	int shortestdist = 6000;
	for (int i = 0; i < cubecnt; i++){
		if(cubes[i].distance < shortestdist){
			shortestdist = cubes[i].distance;
			idx = i;
		}
	}

	double theta = 0, r = 0, rLeft = 0, rRight = 0;
	angle2use = cubes[idx].angle + 6.0; // Shift for angle adjustment issues
	theta = (M_PI * angle2use)/180;
	theta = std::abs(theta);
	r = (cubes[idx].distance * std::sin((M_PI/2) - theta))/(std::sin(2 * theta));
	rRight = ((angle2use > 0) ? (r - 298) : (r + 298));
	rLeft = ((angle2use > 0) ? (r + 298) : (r - 298));
	rightcm = (theta * rRight)/5;
	leftcm = (theta * rLeft)/5;

//	//Bring it back a little
//	rightcm -= 3;
//	leftcm -= 3;

	// Found wasn't grabbing a cube so add a little bit
	rightcm -= 3;
	leftcm -= 3;

	printf("theta = %f, r = %f, rRight = %f, rLeft = %f\n", theta, r, rRight, rLeft);
	printf("rightcm = %i   leftcm = %i\n", rightcm, leftcm);
	}

int Lidar::findCubes(double frangle, double toangle)
	{
	switch(cubeFindCase)
		{
		case 0:
			//Read from lidar
			printf("Reading\n");
			readLidar();
			readcnt = 0;
			cubeFindCase++;
			break;
		case 1:
			//Check for completion
			if(readComplete())
				{
				cubeFindCase++;
				}
			break;
		case 2:
			//Process the data

			printf("Data Count = %i\n",glob_lidar_count);
			filterData(true, 40,40,50,5000);
			FindLines();
			checkLinesForCubes(frangle,toangle);
			if(cubecnt > 0) //Did we find any?
				{
				calculatePathToNearestCube();
				m_calculator1_Ptr.reset(new DriveMotorCalculator(leftcm, rightcm, 33));
				m_calculator1_init = false;
				}
			else
				return 2;

			cubeFindCase++;
			break;
		case 3:

			//Go to Distance here!
			if(moveRobot(m_calculator1_init, m_calculator1_Ptr))
				{
				cubeFindCase = 0;
				printf("Done\n");
				return 1;
				}
			break;
		}
	return 0;
	}

int Lidar::checkForCubeIntake()
	{
	switch (cubeIntakeCase)
		{
		case 0:
			//Check if the intake is in position
			//if(Robot::intake->checkPosition() != Robot::intake->IntakePositions::GetCube)
			//	return 2;
			//Get readings
			readLidar();
			cubeIntakeCase = 1;
			break;
		case 1:
			//Check for completion
			if(readComplete())
				{
				cubeIntakeCase = 2;
				}
			break;
		case 2:
			{
			filterData(true, 40,40,50,500);
			FindLines();
			//When we filter out everything we will be left with only whats in between the intakes
			//Is there something here?
			if((linecnt+1) < 1)
				return 2; //No cube!
			//There must be something here then
			//	What is the largest line
			int larLen = 0;
			int stLine = -1;
			for(int i = 0; i<(linecnt+1);i++)
				{
				if(lines[i].length > 50)
					{
					if(stLine == -1)
						stLine = i;
					}
				if(lines[i].length > larLen)
					larLen = lines[i].length;
				}
			if (larLen < 30)
				return 2; //No cube!
			//By now I think we can be real sure that there is a cube in front of the robot
			//	Is is flat or diagonal?
			//	Diagonal will look like V flat looks like _
			//1 or 2 lines?
			if ((linecnt+1) > 1)
				{
				//We may have a diagonal
				// Is the difference in angles around 90 degrees?
				//does the other line exist?
				if((stLine+1) > (linecnt+1))
					stLine--;
				if(std::abs(std::abs((lines[stLine].angle - lines[stLine+1].angle)) - 90) < 10)
					return 3; //Return diamond shape
				}
			else
				{
				//Check angle of the line
				if(std::abs(lines[stLine].angle - 0) > 20)
					return 4; //Not diamond, but also not quite lined up right
				}
			//At this point the cube is flat
			//Check distance
			int xLoca = (lines[stLine].start.x + lines[stLine].end.x) / 2;
			int yLoca = (lines[stLine].start.y + lines[stLine].end.y) / 2;
			int dist = std::sqrt((xLoca*xLoca) + (yLoca*yLoca));
			//	If the distance is close enough then the cube is ready to be picked up
			if(dist < 255)
				return 1;//Cube flat and ready for pickup!
			else
				return 5;//Cube too far away
			cubeIntakeCase = 0;
			break;
			}
		}
	return 0;
	}

int Lidar::squareUpCube()
	{
	switch(cubeSquaringCase)
		{
		case 0:
			//Check if the intake is in position
			if(Robot::intake->Status.position != Robot::intake->IntakePositions::GetCube)
				return 2;
			//Get readings
			readLidar();
			cubeSquaringCase = 1;
			break;
		case 1:
			//Check for completion
			if(readComplete())
				{
				cubeSquaringCase = 2;
				printf("Got readings\n");
				}
			break;
		case 2:
			{
			filterData(true, 40,40,50,300);
			FindLines();
			//When we filter out everything we will be left with only whats in between the intakes
			//Is there something here?
			if((linecnt+1) < 1)
				return 2; //No cube!
			//There must be something here then
			//	What is the largest line
			int larLen = 0;
			int stLine = -1;
			for(int i = 0; i<(linecnt+1);i++)
				{
				if(lines[i].length > 50)
					{
					if(stLine == -1)
						stLine = i;
					}
				if(lines[i].length > larLen)
					larLen = lines[i].length;
				}
			if (larLen < 30)
				{
				return 2; //No cube!
				}
			printf("idx = %i\n",stLine);
			//By now I think we can be real sure that there is a cube in front of the robot
			//	Is is flat or diagonal?
			diagonal = false;
			spinCW = false;
			//	Diagonal will look like V flat looks like _
			//1 or 2 lines?
			if ((linecnt+1) > 1)
				{
				//We may have a diagonal
				// Is the difference in angles around 90 degrees?
				//does the other line exist?
				if((stLine+1) > (linecnt+1))
					stLine--;
				if(std::abs(std::abs((lines[stLine].angle - lines[stLine+1].angle)) - 90) < 10)
					{
					//It is diagonal!
					diagonal = true;
					//Which way do we spin the wheels?
					if(lines[stLine].length > lines[stLine+1].length)
						spinCW = true;
					count = 0;
					cubeSquaringCase = 3;
					int dist = std::sqrt((lines[stLine].start.x*lines[stLine].start.x) + (lines[stLine].start.y*lines[stLine].start.y));
					printf("dist = %i\n",dist);
					if(dist > 150)
						{
						count = 0;
						cubeSquaringCase = 5;
						}
					return 0;
					}
				}
			else
				{
				//Check angle of the line
				printf("One Line: ang=%f\n",lines[stLine].angle);
				if(std::abs(lines[stLine].angle - 0) > 20)
					{
					//It is diagonal!
					diagonal = true;
					if(lines[stLine].angle < 0)
						spinCW = true;
					count = 0;
					cubeSquaringCase = 3;
					return 0;
					}
				}
			//At this point the cube is flat
			//Check distance
			int xLoca = (lines[stLine].start.x + lines[stLine].end.x) / 2;
			int yLoca = (lines[stLine].start.y + lines[stLine].end.y) / 2;
			int dist = std::sqrt((xLoca*xLoca) + (yLoca*yLoca));
			printf("dist = %i\n",dist);
			//	If the distance is close enough then the cube is ready to be picked up
			if(dist < 60)
				{
				cubeSquaringCase = 6;
				return 0;
				}
			if(dist > 170)
				{
				count = 0;
				cubeSquaringCase = 5;
				return 0;
				}
			count = 0;
			cubeSquaringCase = 4; //Cube is flat
			//We now know what we need to do
			break;
			}
		case 3:
			count++;
			if(diagonal)
				{
				printf("Diagonal\n");
				//Attempt to unclamp, turn a little
				Robot::intake->release();
				if(spinCW)
					{
					//Give power to left side (reverse)
					Robot::intake->setRightSpeed(0.7);
					Robot::intake->setLeftSpeed(0);
					Robot::driveTrain->SetLeftPower(0);
					Robot::driveTrain->SetRightPower(-0.6);
					printf("Turning CW\n");
					}
				else
					{
					//Give power to right side (reverse)
					Robot::intake->setLeftSpeed(0.7);
					Robot::intake->setRightSpeed(0);
					Robot::driveTrain->SetLeftPower(-0.6);
					Robot::driveTrain->SetRightPower(0);
					printf("Turning CCW\n");
					}
				}
			else
				{
				printf("Not Diagonal");
				count = 0;
				cubeSquaringCase = 4; //We should re-check the cube
				}
			if(count == 15)
				{
				count = 0;
				cubeSquaringCase = 4; //We should re-check the cube
				}
			break;
		case 4:
			count++;
			//The cube is flat now just take it in all the way
			printf("Bring In\n");
			Robot::intake->grab();
			Robot::driveTrain->SetLeftPower(0);
			Robot::driveTrain->SetRightPower(0);
			Robot::intake->spinForward(0.7);
			//Set power of left and right to be inwards
			if (count == 20)
				cubeSquaringCase = 0; //Double check the cube
			break;
		case 5:
			//We lost it
			//Open and drive forward a little
			count++;
			Robot::intake->release();
			Robot::driveTrain->SetLeftPower(0.4);
			Robot::driveTrain->SetRightPower(0.4);
			Robot::intake->spinForward(0.7);
			if(count > 15)
				{
				Robot::driveTrain->SetLeftPower(0);
				Robot::driveTrain->SetRightPower(0);
				Robot::intake->spinForward(0);
				cubeSquaringCase = 0;
				}
			break;
		case 6:
			printf("Done\n");
			Robot::intake->stopWheels();
			cubeSquaringCase = 0;
			return 1;
			break;
		}
	return 0;
	}
// Put methods for controlling this subsystem
// here. Call these from Commands.


void  Lidar::getDistanceToCube(int &leftDistCm, int &rightDistCm) {
	leftDistCm  = leftcm;
	rightDistCm = rightcm;
}
