package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class Report {
    private long reportId; // 신고ID(PK)
    private long stallId; // 화장실칸ID
    private String content; // 신고내용
    private int userReportReason; // 신고사유
    private int checked; // 읽음(1), 안읽음(0) 표시
}