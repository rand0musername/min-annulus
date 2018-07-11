#pragma once
#include "beach_line.h"
#include "geometry.h"

class LeafNode;

class Event {
   public:
    Event(double x, double y, char type);
    double GetY() const { return y; }
    char GetType() const { return type; }

    bool operator<(const Event& b) const {
        if (y == b.y) {
            return x < b.x;
        }
        return y < b.y;
    };

   private:
    double x, y;
    char type;  // 'c' or 's'
};

class CircleEvent : public Event {
   public:
    CircleEvent(double y, geometry::Point center, LeafNode* arc);

    LeafNode* GetArc() const { return arc; }
    geometry::Point GetCenter() const { return center; }
    void SetArc(LeafNode* arc) { this->arc = arc; }
    void SetFalseAlarm(bool false_alarm) { this->false_alarm = false_alarm; }
    bool GetFalseAlarm() const { return false_alarm; }

   private:
    geometry::Point center;
    LeafNode* arc;
    bool false_alarm;
};

class SiteEvent : public Event {
   public:
    SiteEvent(double x, double y, int site);
    int GetSite() const { return site; }

   private:
    int site;
};
