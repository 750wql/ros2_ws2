//
// Created by lbw on 25-1-4.
//

#ifndef JUDGER_DETAILS_H
#define JUDGER_DETAILS_H
typedef struct Details {
    float x;
    float y;
    bool operator <(const Details & a) const
    {
        return x<a.x;
    }
}CityDetails;


#endif //JUDGER_DETAILS_H
