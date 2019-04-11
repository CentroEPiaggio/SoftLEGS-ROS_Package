#include <omd/opto.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <QtCore/QCoreApplication>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <string.h>
using namespace std;


        int main(int argc, char *argv[])
        {
            QCoreApplication a(argc,argv);

            OptoDAQ daq;
            OptoPorts ports;

            OPort* portlist = ports.listPorts(true);

            //std::cout << "Test!" << std::endl;

            if (ports.getLastSize() == 0)
            {
                std::cout << "No sensor available!" << std::endl;
                return 0;
            }


            daq.open(portlist[0]);
            daq.zeroAll();

            OptoPackage* pa=0;

            int size = daq.readAll(pa,false);


            while(1)
            {

                std::cout<<"test"<<std::endl;


                for (int k=0;k<daq.getSensorSize();k++)
                {
                    std::cout<<"Sensor "<<k<<" datas:"<<endl;;
                    std::cout<<"(Size: "<<size<<")"<<endl;;
                    for (int i=0;i<size;i++)
                    {
                        std::cout<<"x: "<<pa[k*size+i].x<<" y: "<<pa[k*size+i].y<<" z: "<<pa[k*size+i].z<<" s1: "<<pa[k*size+i].s1<<" s2: "<<pa[k*size+i].s2<<" s3: "<<pa[k*size+i].s3<<" s4: "<<pa[k*size+i].s4<<" TEMP: "<<pa[k*size+i].temp<<std::endl;
                    }
                }



            }



            daq.close();

            char key;
            std::cin>>key;

            return a.exec();

        }
