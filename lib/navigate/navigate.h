#pragma once

#include <math.h>
#include <iostream>

// linked list of waypoints
typedef struct sPosition {
    double lat;
    double lon;
    struct sPosition *next;
} Position;

//#define PI 3.14159265359

/**
 * Produce beizier noise from a sequence of random numbers.
 * Each all to next delivers the next in the sequence of numbers.
 * The noise will vary from -1 to +1.0, but not be distributed uniformly
 * over the range. 
 * The bezier curve tries to be consistent over C0,C1 & C2, but becomes
 * unstable trending towards -infinity +infinity, hence the control points are
 * scaled. This makes C1 & C2 not perfect, but its better than instability.
 */ 

class BezierNoise {
public:
    BezierNoise() {
        t = 0.0;
        p[0] = (1.0*rand())/(1.0*RAND_MAX);
        p[1] = (1.0*rand())/(1.0*RAND_MAX);
        p[2] = (1.0*rand())/(1.0*RAND_MAX);
        p[3] = (1.0*rand())/(1.0*RAND_MAX);
        n = 0;
        period = (0.1*rand())/(1.0*RAND_MAX);

    }
    double next() {
        t = t+period;
        if (t >= 1.0 ) {
            // P3 = Q0
            // Q0 = P3
            // Q1 = Q0 - P2 + P3
            // Q2 = P1 - 2*P2 + P3 - Q0 +2Q1
            double p1 = p[1], p2 = p[2], p3 = p[3];
            p[0] = p3;
            p[1] = (p[0]-p2+p3)/p1;
            p[2] = (p1 - 2*p2 + p3 - p[0] + 2*p[1])/p2;
            p[3] = (1.0*rand())/(1.0*RAND_MAX);
            period = 0.01+(0.02*rand())/(1.0*RAND_MAX);
            t = 0.0;
        } 


        double v = pow(1.0-t,3)*p[0] 
            + 3*pow(1.0-t,2)*t*p[1] 
            + 3*pow(t,2)*(1.0-t)*p[2] 
            + pow(t,3)*p[3];
        //std::cout << n << "," << t << "," << v << std::endl;
        n++;
        return v;
    }
private:
    unsigned long n;
    double t;
    double p[4];
    double period;
};

class Boat {
public:
    Boat() {
    };

    void setRoute(Position *waypoint) {
        this->waypoint = waypoint;
        pos.lat = waypoint->lat;
        pos.lon = waypoint->lon;
        dtw = 1000000000;        
    }
    // Rumb line distance and bearing aproximated for short distances 
    // ignoring mercator projection.
    bool navigate(unsigned long elapsed) {
        bool ret = false;
        distanceTravelled = sog * 0.001*elapsed;
        if ( dtw < 100.0 || distanceTravelled > dtw ) {
            // arrived.
                waypoint = waypoint->next;
            ret = true;
        }
        // update position.
        updatePosition(&pos,  cog, distanceTravelled);
        dtw = distanceBetween(&pos, waypoint);
        btw = bearing(&pos, waypoint);
        cog = btw + 5.0*noise.next()*PI/180.0;
        return ret;
    }

    double distanceBetween(Position *wp1, Position *wp2) {
        double lat1 = wp1->lat*PI/180.0;
        double lat2 = wp2->lat*PI/180.0;
        double lon1 = wp1->lon*PI/180.0;
        double lon2 = wp2->lon*PI/180.0;
        double dLatR = lat2 - lat1;
        double dLonR = lon2 - lon1;
        double sinLatR = sin(dLatR/2);
        double sinLonR = sin(dLonR/2);
        // d = 2R × sin⁻¹(√[sin²((θ₂ - θ₁)/2) + cosθ₁ × cosθ₂ × sin²((φ₂ - φ₁)/2)])
        return 2.0*6378100.0 * asin(sqrt(sinLatR*sinLatR + cos(lat1)*cos(lat2) * sinLonR * sinLonR));
    }
    double bearing(Position *wp1, Position *wp2) {
        double lat1 = wp1->lat*PI/180.0;
        double lat2 = wp2->lat*PI/180.0;
        double lon1 = wp1->lon*PI/180.0;
        double lon2 = wp2->lon*PI/180.0;
        double dLonR = lon2 - lon1;

        double x = cos(lat2) * sin(dLonR);
        // Y = cos θa * sin θb – sin θa * cos θb * cos ∆L
        double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLonR);
        double b = atan2(x,y);
        if ( b < 0 ) b = b+PI*2;
        return b;
    }
    void updatePosition(Position *pos, double cog, double distance) {
        double lat1 = pos->lat*PI/180.0;
        double lon1 = pos->lon*PI/180.0;
        double ad = distance/6378100.0;
        log = log + distance;

        //latitude of second point = la2 =  asin(sin la1 * cos Ad  + cos la1 * sin Ad * cos θ), and
        //longitude  of second point = lo2 = lo1 + atan2(sin θ * sin Ad * cos la1 , cos Ad – sin la1 * sin la2)

        double lat2 = asin( sin(lat1) * cos(ad) + cos(lat1) * sin(ad) * cos(cog));
        double lon2 = lon1 + atan2(sin(cog)*sin(ad) * cos(lat1), cos(ad) - sin(lat1)*sin(lat2));
        if ( lat2 > PI/2) {
            lat2 = PI - lat2; 
        } else if (lat2 < -PI/2) {
            lat2 = lat2-PI;
        }
        if ( lon2 > PI) {
            lon2 = lon2-2*PI;
        } else if ( lon2 < -PI ) {
            lon2 = lon2+2*PI;

        }
        //std::cout << "Update possition distance:" << distance << " cog:" << cog << " lat:"  << pos->lat << "->" << lat2*180.0/PI << " lon:" << pos->lon << "->" << lon2*180.0/PI << std::endl;
        pos->lat = lat2*180.0/PI;
        pos->lon = lon2*180.0/PI;

    }


    double sog;
    double cog;
    double btw;
    double dtw;
    double log;
    double distanceTravelled;
    Position pos;


private:
    Position *waypoint;
    BezierNoise noise;
    unsigned long lastFix;

};




