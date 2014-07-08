#ifndef OFXSMOOTHPOINT_H_INCLUDED
#define OFXSMOOTHPOINT_H_INCLUDED

/// Сглаживание значений во времени

/// Для трехмерной точки
class smoothPoint3f {

 public:
   ofVec3f result;
   unsigned int historySize;

   void setup(unsigned int historySize){
    this->historySize = historySize;
    it = 0;
   }

   ofVec3f getSmoothValue(ofVec3f p)
   {
    it++;
    if (it > historySize-1) it = 0;
    if (history_points.size() < historySize) history_points.push_back(p);
     else history_points[it] = p;

    for (unsigned int i = 0; i < history_points.size();i++)
     result += history_points[i];
     result /= history_points.size()+1;
     return result;
   }

   void clear(){
    history_points.clear();
   }


   void insertData(ofVec3f p)
   {
    history_points.clear();
    for (int i = 0; i < historySize;i++)
     history_points.push_back(p);
   }

 private:
    vector <ofVec3f> history_points;
    unsigned int it;
};

/// Для двумерной точки
class smoothPoint2f {

 public:
   ofVec2f result;
   unsigned int historySize;

   void setup(unsigned int historySize){
    this->historySize = historySize;
    it = 0;
   }

   ofVec2f getSmoothValue(ofVec2f p)
   {

    it++;

    if (it > historySize-1) it = 0;
    if (history_points.size() < historySize) history_points.push_back(p);
     else history_points[it] = p;

    for (unsigned int i = 0; i < history_points.size();i++)
     result += history_points[i];
     result /= history_points.size()+1;
     return result;
   }

   void clear(){
    history_points.clear();
   }

   void insertData(ofVec2f p)
   {
    history_points.clear();
    for (int i = 0; i < historySize;i++)
     history_points.push_back(p);
   }


 private:
    vector <ofVec2f> history_points;
    unsigned int it;
};

/// Для одномерного значения
class smoothFloat {

 public:

   float result;

   void setup(unsigned int historySize){
    this->historySize = historySize;
    it = 0;
   }

   float getSmoothValue(float p)
   {
    it++;
    if (it > historySize-1) it = 0;
    if (history_points.size() < historySize) history_points.push_back(p);
     else history_points[it] = p;

    for (unsigned int i = 0; i < history_points.size();i++)
     result += history_points[i];
     result /= history_points.size()+1;
     return result;
   }

 private:
    vector <float> history_points;
    unsigned int historySize;
    unsigned int it;
};

#endif // OFXSMOOTHPOINT_H_INCLUDED
