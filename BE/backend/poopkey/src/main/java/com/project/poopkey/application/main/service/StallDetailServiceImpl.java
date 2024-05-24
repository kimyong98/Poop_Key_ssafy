package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.StallDetailDao;
import com.project.poopkey.application.main.dto.StallDetail;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class StallDetailServiceImpl implements  StallDetailService{
    @Autowired
    private StallDetailDao stallDetailDao;

    @Override
    public StallDetail findStallDetail(long stallId) {
        return stallDetailDao.selectOne(stallId);
    }
}
