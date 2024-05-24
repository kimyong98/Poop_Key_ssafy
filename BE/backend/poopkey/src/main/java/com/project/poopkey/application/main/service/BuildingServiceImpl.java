package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.BuildingDao;
import com.project.poopkey.application.main.dto.Building;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class BuildingServiceImpl implements BuildingService{
    @Autowired
    private BuildingDao buildingDao;

    @Override
    public List<Building> findAllBuilding() {
        return buildingDao.selectAll();
    }
}
