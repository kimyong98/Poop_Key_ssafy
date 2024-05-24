package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.StallDao;
import com.project.poopkey.application.main.dto.Stall;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class StallServiceImpl implements StallService{
    @Autowired
    private StallDao stallDao;

    @Override
    public Stall findStall(long stallId) {
        return stallDao.selectOne(stallId);
    }


}
