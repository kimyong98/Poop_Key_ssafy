package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.MasterDao;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class MasterServiceImpl implements MasterService{
    @Autowired
    private MasterDao masterDao;

    @Override
    public void removeMaster(String email) {
        masterDao.delete(email);
    }
}
