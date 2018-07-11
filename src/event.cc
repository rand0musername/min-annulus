#include "event.h"

Event::Event(double x, double y, char type) {
    this->x = x;
    this->y = y;
    this->type = type;
}

CircleEvent::CircleEvent(double y, geometry::Point center, LeafNode* arc) : Event(center.x, y, 'c') {
    this->center = center;
    this->arc = arc;
    this->false_alarm = false;
}

SiteEvent::SiteEvent(double x, double y, int site) : Event(x, y, 's') { this->site = site; }
