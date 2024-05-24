import { useNavigate, useParams } from "react-router-dom";
import { useState } from "react";
import { createReportData } from "../../util/API";
import styles from "./ToiletReport.module.css";
import ReportItem from "./ReportItem";
import Swal from "sweetalert2";

const reportList = [
  {
    id: 0,
    report_img: process.env.PUBLIC_URL + `/assets/dirty.png`,
    report_descript: "위생",
  },
  {
    id: 1,
    report_img: process.env.PUBLIC_URL + `/assets/broken.png`,
    report_descript: "파손",
  },
  {
    id: 2,
    report_img: process.env.PUBLIC_URL + `/assets/more.png`,
    report_descript: "기타",
  },
];
const ToiletReport = () => {
  // 화장실칸 id, 화장실 id, content (Null), 신고사유 (select 태그 이용 위생:0, 파손:1, 기타:2)
  const [reason, setReason] = useState(0);
  const [content, setContent] = useState("");
  const { buildingId, stallId, floor } = useParams();
  const navigate = useNavigate();

  const handleClickEmotion = (reason) => {
    setReason(reason);
  };

  const handleSubmit = () => {
    const data = {
      stallId: stallId, // 화장실칸ID
      content: content, // 신고내용
      userReportReason: reason, // 신고사유 (0: 위생, 1: 파손, 2: 기타)
    };
    createReportData(data);
    Swal.fire({
      icon: "success",
      title: "신고 완료!",
      text: `신고 접수가 완료되었습니다.`,
      showCancelButton: false,
      confirmButtonText: "확인",
    });
    navigate(`/toilet/${buildingId}`, { state: { floor: floor } });
  };

  return (
    <div className={styles.container}>
      <div className={styles.logoContainer}>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
        />
        <div className={styles.text}>User Report Page</div>
      </div>
      <div className={styles.reportContainer}>
        <section>
          <div className={styles.reportType}>신고사유</div>
          <div className={styles.report_list_wrapper}>
            {reportList.map((it) => (
              <ReportItem
                key={it.id}
                {...it}
                onClick={handleClickEmotion}
                //isSelected -> 선택된 아이템만 true -> style을 위한 변수
                isSelected={it.id === reason}
              />
            ))}
          </div>
        </section>
        <section>
          <div className={styles.reportType}>신고내용</div>
          <div>
            <textarea
              className={styles.report_content}
              value={content}
              onChange={(e) => setContent(e.target.value)}
            />
          </div>
        </section>
        <section className={styles.buttonContainer}>
          <div className={styles.button} onClick={handleSubmit}>
            제출하기
          </div>

          <div
            className={styles.button}
            onClick={() =>
              navigate(`/toilet/${buildingId}`, { state: { floor: floor } })
            }
          >
            취소하기
          </div>
        </section>
      </div>
    </div>
  );
};

export default ToiletReport;
