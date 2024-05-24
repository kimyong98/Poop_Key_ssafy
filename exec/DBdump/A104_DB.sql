-- 공통프로젝트 PoopKey DATABASE --
-- ver 1.1.1
-- 수정사항 : 모든 테이블의 PK에 auto_increment 직접추가
-- ver 1.1.2
-- 수정사항 : item 테이블의 tissue_status column 제거(SQL에서 계산되므로 불필요함)
-- ver 1.1.3
-- 수정사항 : user 테이블의 비밀번호, 이메일 NOT NULL로 바꿈

-- ver 2.1.1
-- 수정사항 :
-- 사용자 테이블에서 삭제할 column들 휴대폰번호, 이름, 사진, 주소
-- 관리자 column은 not null로 하기
-- 탈퇴일을 추가

-- 블록 table의 내용들을 stall table로 합치고
-- 화장실 칸 입력을 받을 때는 NOT NULL column에 DEFAULT 0을 추가해서 빈 칸으로 json을 보내도 자동으로 0이 입력되게 만든다.

-- stall table 의 stall_number column 제거

-- statistics라는 table 신설
-- stall table의 cleaning을 점검중 여부로 바꾸기
-- stall table의 able column 을 남기는지 문제는 더 생각해보기=>팀장님 말대로 제거하는 쪽으로

-- report table에 신고 처리된 시간을 기록하는 timestamp column을 추가

-- 사용자 관련 테이블의 bigint를 전부 int로 교체
-- bigint(20)에서 숫자 제한 없애기 (bigint로 교체)

-- 분산돼있던 통계 관련 column statistic table로 모두 옮기기 

-- 신고 table의 화장실ID column 제거(반정규화 제거)
-- ERD 모든 column에 초기값 설정

-- ver 2.1.2
-- 수정사항 : toilet table 제거하고 column 내용을 stall table에 합침

-- ver 2.1.3
-- 수정사항 : user table 생성일 column DEFAULT CURRENT_TIMESTAMP 추가

-- ver 2.1.4
-- 수정사항 : report table에 신고확인여부 checked column 추가


DROP SCHEMA IF EXISTS `poopkey`;
-- -----------------------------------------------------
-- Schema poopkey
-- -----------------------------------------------------

CREATE SCHEMA IF NOT EXISTS `poopkey` DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci ;
USE `poopkey` ;

CREATE TABLE `master` (
	`master_id`	int	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`password`	varchar(255)	NOT NULL	COMMENT 'hash한 결과값을 저장해야 한다.',
	`name`	varchar(100)	NOT NULL,
	`phone`	varchar(20)	NOT NULL,
	`device_id`	varchar(255)	NOT NULL	DEFAULT 'no_content',
	`created_date`	timestamp	NOT NULL	DEFAULT CURRENT_TIMESTAMP	COMMENT '회원 가입 날짜',
	`email`	varchar(255)	NOT NULL	COMMENT '이메일 주소가 아이디 역할',
	`profile`	longblob	NULL	COMMENT '이미지파일을 binary형태로 DB에 저장, 기본이미지 파일 준비하기',
	`address`	varchar(255)	NOT NULL	DEFAULT 'no_content',
	`deleted_date`	timestamp	NULL	COMMENT '회원 탈퇴 날짜'
);
CREATE TABLE `restroom` (
	`restroom_id`	int	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`building_id`	int	NOT NULL,
	`floor`	int	NOT NULL	DEFAULT 1,
	`congestion`	int	NOT NULL	DEFAULT 0	COMMENT '0:원활, 1:혼잡, 2:포화',
	`gender`	int	NOT NULL	DEFAULT 0	COMMENT '0:남성, 1:여성',
	`cleaning`	int	NOT NULL	DEFAULT 0	COMMENT '0:청소x, 1:청소중',
	`height`	int	NOT NULL	DEFAULT 1	COMMENT '인덱스 번호 1부터 시작',
	`width`	int	NOT NULL	DEFAULT 1	COMMENT '인덱스 번호 1부터 시작',
	`last_cleaning_time`	timestamp	NOT NULL	DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE `building` (
	`building_id`	int	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`master_id`	int	NOT NULL,
	`building_name`	varchar(255)	NOT NULL	DEFAULT 'no_content',
	`building_lat`	double	NULL,
	`building_lng`	double	NULL,
	`congestion`	int	NOT NULL	DEFAULT 0	COMMENT '0:원활, 1:혼잡, 2:포화',
	`address`	varchar(255)	NOT NULL	DEFAULT 'no_content'
);

CREATE TABLE `report` (
	`report_id`	bigint	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`stall_id`	bigint	NOT NULL,
	`content`	varchar(255)	NULL,
	`created_date`	timestamp	NOT NULL	DEFAULT CURRENT_TIMESTAMP,
	`user_report_reason`	int	NOT NULL	DEFAULT 0	COMMENT '위생:0, 파손:1, 기타:2  셋 중 하나를 선택',
	`handled_date`	timestamp	NULL	DEFAULT NULL	COMMENT '신고가 처리된 날짜 기록',
    `checked` int NOT NULL DEFAULT 0
);

CREATE TABLE `stall` (
	`stall_id`	bigint	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`restroom_id`	int	NOT NULL,
	`status`	int	NOT NULL	DEFAULT 0	COMMENT '사용가능:0, 사용중:1, 고장:2, 점검:3, 고장의심:4',
	`last_used_time`	timestamp	NOT NULL	DEFAULT CURRENT_TIMESTAMP	COMMENT '화장실을 사용했을 때 갱신',
	`closed`	int	NOT NULL	DEFAULT 0	COMMENT '0:열림, 1:잠김',
	`used_number_after_cleaning`	int	NOT NULL	DEFAULT 0	COMMENT '청소 후 0 으로 reset',
	`maintaining`	int	NOT NULL	DEFAULT 0	COMMENT '0:점검x, 1:점검중',
    `row`	int	NOT NULL	DEFAULT 1	COMMENT '인덱스 번호 1부터 시작',
	`col`	int	NOT NULL	DEFAULT 1	COMMENT '인덱스 번호 1부터 시작',
	`content`	int	NOT NULL	DEFAULT 0	COMMENT '0:빈칸, 1:화장실, 2:입구',
    `toilet_status`	int	NOT NULL	DEFAULT 0	COMMENT '변기 막힘 여부는 화장실 칸 table의 고장 column으로 반영된다. 0:변기막힘 x, 1:변기막힘'
);


CREATE TABLE `item` (
	`item_id`	bigint	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`stall_id`	bigint	NOT NULL,
	`tissue`	int	NOT NULL	DEFAULT 0	COMMENT '0부터 100까지 정수로 퍼센트 표시'
);

CREATE TABLE `user` (
	`user_id`	int	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`password`	varchar(255)	NOT NULL,
	`email`	varchar(255)	NOT NULL	COMMENT '이메일 주소가 아이디 역할',
	`created_date`	timestamp	NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE `bookmark` (
	`bookmark_id`	bigint	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`user_id`	int	NOT NULL,
	`created_date`	timestamp	NOT NULL	DEFAULT CURRENT_TIMESTAMP,
	`content`	varchar(255)	NOT NULL
);

CREATE TABLE `statistic` (
	`statistic_id`	bigint	NOT NULL PRIMARY KEY AUTO_INCREMENT,
	`stall_id`	bigint	NOT NULL,
	`used_number`	int	NOT NULL	DEFAULT 0,
	`last_clean_date`	timestamp	NOT NULL	DEFAULT CURRENT_TIMESTAMP,
	`tissue_change_number`	int	NOT NULL	DEFAULT 0,
	`break_number`	int	NOT NULL	DEFAULT 0
);

-- DEBUGGING
SELECT * FROM user;
SELECT * FROM master;
-- DUMMY DATA
INSERT INTO master VALUES
(1, '1234', '김싸피', '010-1234-5678', 'XYZ1234', CURRENT_TIMESTAMP, 'ssafy@ssafy.com',
NULL, '서울시 강남구 테헤란로 212', NULL);
INSERT INTO building VALUES
(1, 1, 'SSAFY 서울 캠퍼스', 37.5012851921, 127.0396046633, 0, '서울시 강남구 테헤란로 212');
INSERT INTO restroom VALUES
(1, 1, 1, 0, 0, 0, 3, 3, CURRENT_TIMESTAMP), -- SSAFY빌딩 1층 남자화장실
(2, 1, 1, 1, 1, 0, 3, 3, CURRENT_TIMESTAMP), -- SSAFY빌딩 1층 여자화장실
(3, 1, 2, 0, 0, 0, 3, 3, CURRENT_TIMESTAMP), -- SSAFY빌딩 2층 남자화장실
(4, 1, 2, 1, 1, 1, 3, 3, CURRENT_TIMESTAMP); -- SSAFY빌딩 2층 여자화장실

INSERT INTO stall VALUES
-- -- 1층 남자화장실 칸
-- 3, 5, 6, 7번째 => 화장실 상태관련 정보 
(1, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 1, 1, 1, 0), -- 화장실칸
(2, 1, 1, CURRENT_TIMESTAMP, 1, 1, 0, 1, 2, 1, 0), -- 화장실칸
(3, 1, 0, CURRENT_TIMESTAMP, 0, 2, 0, 1, 3, 1, 0), -- 화장실칸
(4, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 1, 0, 0), -- 빈칸
(5, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 2, 0, 0), -- 빈칸
(6, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 3, 0, 0), -- 빈칸
(7, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 1, 0, 0), -- 빈칸
(8, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 2, 0, 0), -- 빈칸
(9, 1, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 3, 2, 0), -- 입구
-- -- 1층 여자화장실 칸
(10, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 1, 1, 1, 0), -- 화장실칸
(11, 2, 2, CURRENT_TIMESTAMP, 0, 3, 0, 1, 2, 1, 1), -- 화장실칸
(12, 2, 1, CURRENT_TIMESTAMP, 1, 4, 0, 1, 3, 1, 0), -- 화장실칸
(13, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 1, 0, 0), -- 빈칸
(14, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 2, 0, 0), -- 빈칸
(15, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 3, 0, 0), -- 빈칸
(16, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 1, 0, 0), -- 빈칸
(17, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 2, 0, 0), -- 빈칸
(18, 2, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 3, 2, 0), -- 입구
-- -- 2층 남자화장실 칸
(19, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 1, 1, 1, 0), -- 화장실칸
(20, 3, 3, CURRENT_TIMESTAMP, 0, 0, 1, 1, 2, 1, 0), -- 화장실칸
(21, 3, 2, CURRENT_TIMESTAMP, 0, 2, 0, 1, 3, 1, 0), -- 화장실칸
(22, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 1, 0, 0), -- 빈칸
(23, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 2, 0, 0), -- 빈칸
(24, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 3, 0, 0), -- 빈칸
(25, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 1, 0, 0), -- 빈칸
(26, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 2, 0, 0), -- 빈칸
(27, 3, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 3, 2, 0), -- 입구
-- -- 2층 여자화장실 칸
(28, 4, 1, CURRENT_TIMESTAMP, 1, 2, 0, 1, 1, 1, 0), -- 화장실칸
(29, 4, 1, CURRENT_TIMESTAMP, 1, 3, 0, 1, 2, 1, 0), -- 화장실칸
(30, 4, 1, CURRENT_TIMESTAMP, 1, 5, 0, 1, 3, 1, 0), -- 화장실칸
(31, 4, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 1, 0, 0), -- 빈칸
(32, 4, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 2, 0, 0), -- 빈칸
(33, 4, 0, CURRENT_TIMESTAMP, 0, 0, 0, 2, 3, 0, 0), -- 빈칸
(34, 4, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 1, 0, 0), -- 빈칸
(35, 4, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 2, 0, 0), -- 빈칸
(36, 4, 0, CURRENT_TIMESTAMP, 0, 0, 0, 3, 3, 2, 0); -- 입구

INSERT INTO item VALUES
(1, 1, 100),
(2, 2, 3), -- 휴지 부족
(3, 3, 95),
(4, 10, 76),
(5, 11, 83),
(6, 12, 4), -- 휴지 부족
(7, 19, 60),
(8, 20, 62),
(9, 21, 2), -- 휴지 부족
(10, 28, 100),
(11, 29, 99),
(12, 30, 71);

INSERT INTO user VALUES
(1, '1234', 'ssafyking@gmail.com', CURRENT_TIMESTAMP),
(2, '5678', 'ssawang@gmail.com', CURRENT_TIMESTAMP);

INSERT INTO report VALUES
(1, 2, '청소가 안 된 부분이 있는 것 같아요', CURRENT_TIMESTAMP, 0, NULL, 0),
(2, 20, '고장이 났어요!!', CURRENT_TIMESTAMP, 1, NULL, 0);

INSERT INTO bookmark VALUES
(1, 1, CURRENT_TIMESTAMP, 'SSAFY 서울 캠퍼스'),
(2, 1, CURRENT_TIMESTAMP, '국립중앙박물관');

INSERT INTO statistic VALUES
(1, 1, 0, CURRENT_TIMESTAMP, 2, 1),
(2, 2, 4, CURRENT_TIMESTAMP, 0, 3),
(3, 3, 5, CURRENT_TIMESTAMP, 1, 2),
(4, 10, 0, CURRENT_TIMESTAMP, 4, 0),
(5, 11, 7, CURRENT_TIMESTAMP, 2, 3),
(6, 12, 13, CURRENT_TIMESTAMP, 1, 0),
(7, 19, 0, CURRENT_TIMESTAMP, 1, 0),
(8, 20, 44, CURRENT_TIMESTAMP, 0, 0),
(9, 21, 2, CURRENT_TIMESTAMP, 1, 0),
(10, 28, 2, CURRENT_TIMESTAMP, 1, 0),
(11, 29, 3, CURRENT_TIMESTAMP, 1, 0),
(12, 30, 5, CURRENT_TIMESTAMP, 1, 0);
-- 모든 더미데이터 생성 끝
SELECT * FROM stall;
SELECT * FROM restroom;
SELECT * FROM item;
SELECT * FROM statistic;
SELECT * FROM report;