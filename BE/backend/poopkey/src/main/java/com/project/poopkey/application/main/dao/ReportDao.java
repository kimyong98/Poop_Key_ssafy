package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.Report;

import java.util.List;

public interface ReportDao {
    public void insert(Report report);

    public List<Report> selectList(int masterId);
    /**관리자의 읽음 표시*/
    public void update(long reportId);

    /** 신고가 접수되면 그 신고에 해당하는 관리자 id를 받아옴*/
    public int selectMasterId(Report report);

    public int convertReportIdToMasterId(long reportId);
}
