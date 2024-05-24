package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dto.Statistic;

public interface StatisticService {
    Statistic findStatistic(String buildingName, int floor, int gender);
}
