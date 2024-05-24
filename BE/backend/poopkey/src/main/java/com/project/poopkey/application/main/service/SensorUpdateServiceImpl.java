package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.ItemSensorUpdateDao;
import com.project.poopkey.application.main.dao.RestroomSensorUpdateDao;
import com.project.poopkey.application.main.dao.StallSensorUpdateDao;
import com.project.poopkey.application.main.dao.StatisticSensorUpdateDao;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
public class SensorUpdateServiceImpl implements SensorUpdateService{

    @Autowired
    private ItemSensorUpdateDao itemSensorUpdateDao;

    @Autowired
    private RestroomSensorUpdateDao restroomSensorUpdateDao;

    @Autowired
    private StallSensorUpdateDao stallSensorUpdateDao;

    @Autowired
    private StatisticSensorUpdateDao statisticSensorUpdateDao;

    @Override
    @Transactional
    public void modifyTissue(long stallId, int tissue) {
        statisticSensorUpdateDao.updateTissue(stallId, tissue);
        itemSensorUpdateDao.update(stallId, tissue);
    }


    @Override
    @Transactional
    public void modifyOccupied(long stallId) {
        stallSensorUpdateDao.updateOccupied(stallId);
        statisticSensorUpdateDao.updateOccupied(stallId);
    }

    @Override
    @Transactional
    public void modifyVacant(long stallId) {
        stallSensorUpdateDao.updateVacant(stallId);
    }

    @Override
    @Transactional
    public void modifyBreak(long stallId) {
        stallSensorUpdateDao.updateBreak(stallId);
        statisticSensorUpdateDao.updateBreak(stallId);
    }

    @Override
    @Transactional
    public void modifyCongestion(int restroomId, int congestion) {
        restroomSensorUpdateDao.updateCongestion(restroomId, congestion);
    }


}
