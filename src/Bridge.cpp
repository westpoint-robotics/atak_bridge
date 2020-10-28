// Bridge.cpp : Defines the entry point for the application.
//

#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include "CoTClient.hpp"

//  CoT Multicast
//ATAK default for SA Multicast 239.2.3.1:6969
//
//MPU5 CoT Default 239.23.12.230:18999

using namespace std;

//double lat1[] = { 40.45771, 40.45913, 40.45989 };
//double lon1[] = { -79.78910, -79.78931, -79.78935 };

//double lat2[] = { 40.45756, 40.45774, 40.46168};
//double lon2[] = { -79.79065, -79.78670, -79.78581};


double lat1[] = { 41.376658, 41.378007, 41.377447 };
double lon1[] = { -73.962455, -73.960790, -73.963333 };

double lat2[] = { 41.379624, 41.380964, 41.379467 };
double lon2[] = { -73.960074, -73.961931, -73.963063 };

int main()
{
    server_ip = "239.2.3.1"
    server_port = 6969
    highland_falls_lat = 41.386748
    highland_falls_lon = -73.956554
    highland_falls_alt = 25
    
    try
    {
        boost::asio::io_service io_service;
        io_service.run();

        AIDTR::CoTClient client(io_service,
            boost::asio::ip::address::from_string(server_ip), server_port);

        AIDTR::CoTClient client1(io_service,
            boost::asio::ip::address::from_string(server_ip), server_port,
            "AIDTR UAV 1", "a-f-G-U-C-V-U-R");

        client.sendPositionReport(lat1[0] , lon1[0], 27);
        client1.sendPositionReport(lat2[0] , lon2[0], 29);

        client.sendContactReport("BTR80 1", "a-h-G-U-C-A-W-W", highland_falls_lat, highland_falls_lon, highland_falls_alt);
        this_thread::sleep_for(std::chrono::milliseconds(1000));

        for (int i = 0; i < 2; i++) {
            int n = 200;
            double deltaLat1 = (lat1[i + 1] - lat1[i]) / n;
            double deltaLon1 = (lon1[i + 1] - lon1[i]) / n;
            double deltaLat2 = (lat2[i + 1] - lat2[i]) / n;
            double deltaLon2 = (lon2[i + 1] - lon2[i]) / n;

            for (int j = 0; j < n; j++) {
                client.sendPositionReport(lat1[i] + j * deltaLat1, lon1[i] + j * deltaLon1, 28);
                client1.sendPositionReport(lat2[i] + j * deltaLat2, lon2[i] + j * deltaLon2, 23);

                this_thread::sleep_for(std::chrono::milliseconds(125));
                if (i == 1) {
                    if (j==n/2)
                        client.sendContactReport("BTR80 1", "a-h-G-U-C-A-W-W", highland_falls_lat, highland_falls_lon, highland_falls_alt);
                    if (j == 2 * n / 3) {
                        client.sendContactReport("Enemy Troup 2", "a-h-G-U-C-I", 40.46087, -79.78682, highland_falls_alt);
                    }
                }
            }

            if (i == 0) {
                client.sendContactReport("Enemy Troup 1", "a-h-G-U-C-I", 40.4616, -79.78755, highland_falls_alt);
            }
        }
        cout << "Sent   " << client.getSendCount() << " messages." << endl
            << "Errors " << client.getErrorCount() << endl;
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    
    return 0;
}
