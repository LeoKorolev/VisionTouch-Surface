#include "blobTracker.h"

//--------------------------------------------------------------------------------
blobTracker::blobTracker() {
	 
	idCounter = 0;
}

//--------------------------------------------------------------------------------
blobTracker::~blobTracker() {}

//--------------------------------------------------------------------------------
void blobTracker::track(vector<ofVec3f>& points, int aliveFrames, float speedThreshold){

    ///Массив с информацией о пользователях на новом кадре
    vector<touch> newTouchList;

    ///Шаг 1. Задаем всем новым блобам - вероятным пользователям id = -1 и копируем данные блоба
    for(unsigned int i = 0; i < points.size(); i++){

        touch t;
        t.id = -1;
		t.blob = ofxCvBlob();
		t.blob.centroid = points[i];
		t.framesToLive = aliveFrames;
        newTouchList.push_back(t);
		newTouchList.back().filter.setup(2,15,1,1);
		newTouchList.back().filter.setState(points[i], 0.1);
    }
    ///Шаг 2. Делаем трекинг блобов с предыдущего кадра - находим среди новых ближайший
    for(unsigned int i = 0; i < touchList.size(); i++){

        int resultIndex = trackKnn(&newTouchList, &(touchList[i].blob), 3);

        if(resultIndex == -1){
			///Защита от промаргивания
			touchList[i].framesToLive--;
			if(touchList[i].framesToLive <= 0)
				touchList[i].id = -1;///Удаление пользователя
		}
        else{///Пользователь найден в списке

            ///Если для нового блоба оказался другой возможный вариант - сравниваем их
            if(newTouchList[resultIndex].id != -1){

                unsigned int j;
                for(j = 0; j < touchList.size(); j++){
                    if(touchList[j].id == newTouchList[resultIndex].id)
                        break;
                }
                if(j == touchList.size()){
                    newTouchList[resultIndex].id = touchList[i].id;
                    touchList[i] = newTouchList[resultIndex];
                }
                ///Сравнение с блобом-"конкурентом"
                else{

                    double x = newTouchList[resultIndex].blob.centroid.x;
                    double y = newTouchList[resultIndex].blob.centroid.y;
                    double xOld = touchList[j].blob.centroid.x;
                    double yOld = touchList[j].blob.centroid.y;
                    double xNew = touchList[i].blob.centroid.x;
                    double yNew = touchList[i].blob.centroid.y;
                    double distOld = (x-xOld)*(x-xOld)+(y-yOld)*(y-yOld);
                    double distNew = (x-xNew)*(x-xNew)+(y-yNew)*(y-yNew);

                    if(distNew < distOld){
                        newTouchList[resultIndex].id = touchList[i].id;
                        touchList[j].id = -1;
                    }
                    else touchList[i].id = -1;
                }
            }
            ///Трекинг прошел без конфликтов
            else newTouchList[resultIndex].id = touchList[i].id;

        }
    }
    ///Шаг 3. Обновление позиций пользователей с предыдущего кадра, удаление тех, которые не были обнаружены
    for(unsigned int i = 0; i < touchList.size(); i++)
	{
		if(touchList[i].id == -1){///Удаление пользователя
			ofNotifyEvent(blobDeleted, touchList[i].blob);
			touchList.erase(touchList.begin() + i, touchList.begin() + i + 1);
			i--;
		}
		else{

			for(unsigned int j = 0; j < newTouchList.size(); j++)
				if(touchList[i].id == newTouchList[j].id){
					///Обновление данных
					ofVec3f lastCentroid = touchList[i].blob.centroid;///Центроид с предыдущего кадра
					touchList[i].blob = newTouchList[j].blob;
					touchList[i].framesToLive = aliveFrames;

					ofVec3f positionDifference;
					positionDifference.set(touchList[i].blob.centroid.x - lastCentroid.x, touchList[i].blob.centroid.y - lastCentroid.y);

					///Фрагмент кода из CCV
					//float posDelta = sqrtf((positionDifference.x*positionDifference.x)+(positionDifference.y*positionDifference.y));

					//int MOVEMENT_FILTERING = 0;
					//float a = 1.0f - 1.0f / expf(posDelta / (1.0f + (float)MOVEMENT_FILTERING*10));
					//users[i].blob.centroid.x = a * users[i].blob.centroid.x + (1-a) * lastCentroid.x;
					//users[i].blob.centroid.y = a * users[i].blob.centroid.y + (1-a) * lastCentroid.y;
					///Конец фрагмента из CCV

					/// Фильтр Калмана
					if(positionDifference.length() > speedThreshold){
						touchList[i].blob.centroid = touchList[i].filter.getCorrect(touchList[i].blob.centroid);
						ofNotifyEvent(blobMoved, touchList[i].blob);
					}
					else touchList[i].blob.centroid = lastCentroid;

					/*touchList[i].blob.D.set(touchList[i].blob.centroid.x - lastCentroid.x, 
                                          touchList[i].blob.centroid.y - lastCentroid.y);*/
				}
		}
	}
    ///Шаг 4. Добавляем новых возможных пользователей
    for(unsigned int i = 0; i < newTouchList.size(); i++){
		if(newTouchList[i].id == -1){

            newTouchList[i].id = idCounter;
            idCounter++;
            touchList.push_back(newTouchList[i]);
			touchList.back().blob.id = touchList.back().id;
			ofNotifyEvent(blobAdded, touchList.back().blob);
		}
	}
}

//--------------------------------------------------------------------------------
vector<touch> & blobTracker::getTouchList(){
	
	return touchList;
}

//--------------------------------------------------------------------------------
int blobTracker::trackKnn(vector<touch> *newTouchList, ofxCvBlob *track, int k)
{
	int resultIndex = -1;
	vector<touch> newUsers = *newTouchList;
    ///Список соседних блобов (индекс в массиве, дистанция до отслеживаемого блоба)
	list< pair<int, double> > neighbours;
	list< pair<int, double> >::iterator iter;

	double xNew, yNew, xToTrack, yToTrack, dist;
	for(int i=0; i<newUsers.size(); i++)
	{
		xNew = newUsers[i].blob.centroid.x;
		yNew = newUsers[i].blob.centroid.y;

		xToTrack = track->centroid.x;
		yToTrack = track->centroid.y;
		dist = (xNew-xToTrack)*(xNew-xToTrack)+(yNew-yToTrack)*(yNew-yToTrack);

		for(iter = neighbours.begin(); iter != neighbours.end() && dist >= iter->second; iter++);
            if((iter!=neighbours.end())||(neighbours.size()<k)){

                neighbours.insert(iter, 1, std::pair<int, double>(i, dist));
                if(neighbours.size() > k) neighbours.pop_back();
            }
	}

    ///Некая функция голосования, не стал ее менять
	std::map<int, std::pair<int, double> > votes;
	for(iter = neighbours.begin(); iter != neighbours.end(); iter++){

		int count = ++(votes[iter->first].first);
		double dist = (votes[iter->first].second += iter->second);

		if(count > votes[resultIndex].first || count == votes[resultIndex].first
			&& dist < votes[resultIndex].second) resultIndex = iter->first;

	}
	return resultIndex;
}
