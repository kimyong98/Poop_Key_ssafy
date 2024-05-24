package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.ReportDao;
import com.project.poopkey.application.main.dto.Report;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class ReportServiceImpl implements  ReportService{
    @Autowired
    private ReportDao reportDao;

    @Override
    public void addReport(Report report) {
        reportDao.insert(report);
    }

    @Override
    public List<Report> findReportList(int masterId) {
        return reportDao.selectList(masterId);
    }

    @Override
    public void modifyReport(long reportId) {
        reportDao.update(reportId);
    }

    @Override
    public int findMasterId(Report report) {
        return reportDao.selectMasterId(report);
    }

    @Override
    public int convertReportIdToMasterId(long reportId) {
        return reportDao.convertReportIdToMasterId(reportId);
    }
}
