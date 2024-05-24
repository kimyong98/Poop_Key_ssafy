package com.project.poopkey.application.render.service;

import com.project.poopkey.application.render.dao.FloorCongestionDao;
import com.project.poopkey.application.render.dto.FloorCongestion;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class FloorCongestionServiceImpl implements FloorCongestionService{

    @Autowired
    private FloorCongestionDao floorCongestionDao;

    @Override
    public List<FloorCongestion> findFloorCongestionList(int buildingid) {
        return floorCongestionDao.selectList(buildingid);
    }
}
