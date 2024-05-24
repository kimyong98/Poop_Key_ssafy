import styles from "./AdminHome.module.css";
import { useEffect, useState, useRef } from "react";
import { useNavigate, useParams } from "react-router-dom";
import { Web_Socket_URL } from "../../util/API";

const AdminHome = () => {
  const { buildingId } = useParams();
  const [msgCount, setMsgCount] = useState(0);
  const navigate = useNavigate();
  const webSocket = useRef(null);

  // WebSocket
  useEffect(() => {
    // 로그인 하지 않은경우 url 접근을 막는다.
    // if (localStorage.getItem("admin") == null) {
    //   navigate("/admin/login");
    // }

    webSocket.current = new WebSocket(`${Web_Socket_URL}/report`);
    webSocket.current.onopen = () => {
      console.log("웹소켓 연결 성공!!");
      const data = {
        buildingId: 1,
        restroomId: 1,
        masterId: 1, //로컬 스토리지에서 가져온다. or useParams 사용할것
      };
      const jsonData = JSON.stringify(data);
      sendMessage(jsonData);
    };

    webSocket.current.onerror = (error) => {
      console.log("웹소켓 연결 에러:", error);
    };

    webSocket.current.onmessage = (event) => {
      console.log("웹소켓 메시지 수신:", event);
      const mailData = JSON.parse(event.data);
      setMsgCount(mailData.filter((data) => data.checked == 0).length);
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

  // 메시지 보내기
  const sendMessage = (message) => {
    if (webSocket.current.readyState === WebSocket.OPEN) {
      webSocket.current.send(message);
    }
  };

  const handleStatusClick = () => {
    navigate(`/admin/toilet/${buildingId}`);
  };

  const handleRegistClick = () => {
    navigate(`/admin/toiletRegist/${buildingId}`);
  };

  const handleMailBoxClick = () => {
    navigate(`/admin/toiletMailBox`);
  };

  const handleStatisticClick = () => {
    navigate(`/admin/toiletStatistics/${buildingId}`);
  };
  return (
    <div className={styles.container}>
      <div className={styles.logoContainer}>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
        />
        <div className={styles.text}>Admin Home</div>
      </div>
      <div className={styles.selectContainer}>
        <div className={styles.selectText}>Menu</div>
        <div className={styles.selectOption} onClick={handleStatusClick}>
          화장실 현황
        </div>
        <div className={styles.selectOption} onClick={handleRegistClick}>
          화장실 등록
        </div>
        <div className={styles.selectOptionMail} onClick={handleMailBoxClick}>
          화장실 알림
          <span className={msgCount != 0 && styles.badge}>
            {msgCount != 0 ? <>{msgCount}</> : <></>}
          </span>
        </div>
        <div className={styles.selectOption} onClick={handleStatisticClick}>
          화장실 통계
        </div>
      </div>
    </div>
  );
};
export default AdminHome;
