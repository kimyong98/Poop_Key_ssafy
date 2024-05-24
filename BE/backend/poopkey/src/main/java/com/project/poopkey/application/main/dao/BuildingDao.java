package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.Building;

import java.util.List;

public interface BuildingDao {
    public List<Building> selectAll();
}
