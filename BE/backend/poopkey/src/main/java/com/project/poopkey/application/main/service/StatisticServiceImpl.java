package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.StatisticDao;
import com.project.poopkey.application.main.dto.Statistic;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class StatisticServiceImpl implements StatisticService{
    @Autowired
    private StatisticDao statisticDao;

    @Override
    public Statistic findStatistic(String buildingName, int floor, int gender) {
        return statisticDao.selectOne(buildingName, floor, gender);
    }
}
