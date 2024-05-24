package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.*;
import com.project.poopkey.application.main.dto.Restroom;
import com.project.poopkey.application.main.dto.RestroomInfoInsert;
import com.project.poopkey.application.main.dto.StallInfoInsert;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

@Service
public class RestroomServiceImpl implements RestroomService{
    @Autowired
    private RestroomDao restroomDao;

    // 아래부터 새로운 화장실 추가에 들어가는 모든 DAO
    @Autowired
    private ItemDao itemDao;
    @Autowired
    private StatisticDao statisticDao;
    @Autowired
    private StallInfoInsertDao stallInfoInsertDao;

    @Override
    public List<Restroom> findRestroomList(int buildingId) {
        return restroomDao.selectList(buildingId);
    }

    @Transactional
    @Override
    public void addRestroom(RestroomInfoInsert restroomInfoInsert) {
        restroomDao.insertOne(restroomInfoInsert);
        itemDao.insertOne(restroomInfoInsert);
        statisticDao.insertOne(restroomInfoInsert);
        stallInfoInsertDao.insertOne(restroomInfoInsert); // 논리적으로 가장 나중에 나와야 함
    }

    @Transactional
    @Override
    public void modifyRestroomClean(int restroomId) {
        restroomDao.updateRestroomClean(restroomId);
    }
}
