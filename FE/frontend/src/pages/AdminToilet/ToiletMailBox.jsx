import styles from "./ToiletMailBox.module.css";
import React, { useEffect, useState, useRef } from "react";
import { Web_Socket_URL } from "../../util/API";
import { updateMailCheck, getMailLocation } from "../../util/AdminAPI";
import { useNavigate } from "react-router-dom";
import { Pagination } from "@mui/material";

const ToiletMailBox = () => {
  const [mailList, setMailList] = useState();
  const [detail, setDetail] = useState();
  const [mailLocation, setMailLocation] = useState();
  const [mailFloor, setMailFloor] = useState(1);
  const webSocket = useRef(null);
  const navigate = useNavigate();
  //pagenation 변수
  const [page, setPage] = useState(1);
  const [pageData, setPageData] = useState();
  const [lastPage, setLastPage] = useState();

  //pagenation
  useEffect(() => {
    if (mailList == undefined) return;
    if (page == lastPage) {
      setPageData(mailList && mailList.slice(5 * (page - 1)));
    } else {
      setPageData(
        mailList && mailList.slice(5 * (page - 1), 5 * (page - 1) + 5)
      );
    }
  }, [mailList, page]);

  // WebSocket
  useEffect(() => {
    webSocket.current = new WebSocket(`${Web_Socket_URL}/report`);

    webSocket.current.onopen = () => {
      console.log("웹소켓 연결 성공!!");
      const data = {
        buildingId: 1,
        restroomId: 1,
        masterId: 1,
      };

      const jsonData = JSON.stringify(data);
      sendMessage(jsonData);
    };

    webSocket.current.onerror = (error) => {
      console.log("웹소켓 연결 에러:", error);
    };

    webSocket.current.onmessage = (event) => {
      console.log("웹소켓 메시지 수신:", event);
      setMailList(
        JSON.parse(event.data).sort(function (o1, o2) {
          return o2.reportId - o1.reportId;
        })
      );

      console.log(JSON.parse(event.data).length);
      setLastPage(
        event.data.length % 8 == 0
          ? parseInt(JSON.parse(event.data).length / 8)
          : parseInt(JSON.parse(event.data).length / 8) + 1
      );
    };

    webSocket.current.onclose = (event) => {
      console.log("웹소켓 연결 종료:", event);
    };

    return () => {
      if (webSocket.current.readyState === WebSocket.OPEN) {
        webSocket.current?.close();
      }
    };
  }, []);

  // websocket 메시지 보내기
  const sendMessage = (message) => {
    if (webSocket.current.readyState === WebSocket.OPEN) {
      webSocket.current.send(message);
    }
  };

  //pagenation에 필요한 데이터를 초기화하는 훅
  useEffect(() => {});

  //message 데이터 가져오기
  const handleCheckedClick = async (reportId) => {
    try {
      await updateMailCheck(reportId);
      const data = {
        masterId: 1, //로컬 스토리지에서 가져온다. or useParams 사용할것
      };
      const jsonData = JSON.stringify(data);
      sendMessage(jsonData);
    } catch (e) {
      console.log(e);
    }
  };

  // message 세부정보 (신고한 화장실 칸 위치정보)
  const handleOnDetailClick = (reportId) => {
    setDetail(reportId);
  };

  // stallId가 주어졌을 때 화장실 위치를 특정한다. (층, 성별, 칸 위치)
  const handleGetLocationClick = async (stallId) => {
    try {
      const response = await getMailLocation(stallId);
      const floor = `${response.floor}층 `;
      const gender = response.gender == 0 ? "남자화장실 " : "여자화장실 ";
      const location = `${findLocation(response.list, stallId)}번 칸 `;
      console.log(floor, gender, location, "화장실 위치 특정");

      setMailFloor(response.floor);
      setMailLocation({
        floor: floor,
        gender: gender,
        location: location,
      });
    } catch (e) {
      console.log(e);
    }
  };

  // 화장실 칸 번호를 리턴하는 함수
  const findLocation = (list, stallId) => {
    let blank = 0;
    for (let i = 0; i < list.length; i++) {
      if (list[i].content == 0) blank++;
      if (list[i].stallId == stallId) {
        return i + 1 - blank;
      }
    }
  };

  // 홈으로
  const handleHomeClick = () => {
    navigate(`/admin/1`);
  };

  // 페이지네이션 핸들러
  const handlePageChange = (event) => {
    const currPage = Number(event.target.outerText);
    setPage(currPage);
  };

  return (
    <div className={styles.container}>
      <div className={styles.logoContainer}>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
          onClick={handleHomeClick}
        />
        <div className={styles.text}>Admin Mail</div>
      </div>
      <div className={styles.mailContainer}>
        <div className={styles.table}>
          <div className={styles.headRow}>
            <div className={styles.cell}>No</div>
            <div className={styles.cell}>신고사유</div>
            <div className={styles.cellContent}>신고내용</div>
          </div>

          {pageData &&
            pageData.map((mail, index) => {
              return (
                <div key={mail.reportId} className={styles.rowBox}>
                  <div
                    className={styles.row}
                    onClick={() => {
                      handleCheckedClick(mail.reportId);
                      handleOnDetailClick(mail.reportId);
                      handleGetLocationClick(mail.stallId);
                    }}
                  >
                    <div className={styles.cell}>
                      {index + 1 + (page - 1) * 5}
                    </div>
                    <div className={styles.cell}>
                      {mail.userReportReason === 0 && "위생"}
                      {mail.userReportReason === 1 && "파손"}
                      {mail.userReportReason === 2 && "기타"}
                    </div>
                    <div className={styles.cellContent}>{mail.content}</div>
                  </div>
                  {detail && detail == mail.reportId && (
                    <div
                      className={styles.state_content}
                      onClick={() =>
                        navigate(`/admin/toilet/1`, {
                          state: {
                            floor: mailFloor,
                          },
                        })
                      }
                    >
                      <img
                        className={styles.contentImg}
                        src={process.env.PUBLIC_URL + `/assets/location.png`}
                      />
                      <span className={styles.contentText}>
                        {mailLocation && mailLocation.floor}
                        {mailLocation && mailLocation.gender}
                        {mailLocation && mailLocation.location}
                      </span>
                    </div>
                  )}
                </div>
              );
            })}
        </div>
        <Pagination
          className={styles.pagination}
          page={page}
          count={lastPage}
          defaultPage={1}
          onChange={handlePageChange}
        />
      </div>
    </div>
  );
};

export default ToiletMailBox;
