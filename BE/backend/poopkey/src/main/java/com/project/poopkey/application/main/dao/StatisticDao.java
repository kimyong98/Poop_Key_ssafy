package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.RestroomInfoInsert;
import com.project.poopkey.application.main.dto.Statistic;

public interface StatisticDao {
    public Statistic selectOne(String buildingName, int floor, int gender);

    public void insertOne(RestroomInfoInsert restroomInfoInsert);
}
