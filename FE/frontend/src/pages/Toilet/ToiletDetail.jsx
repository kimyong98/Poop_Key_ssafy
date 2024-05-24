import styles from "./ToiletDetail.module.css";
import ToiletReport from "../../components/toilet/ToiletReport";

const ToiletDetatil = () => {
  return (
    <>
      <ToiletReport />
    </>
  );
};
export default ToiletDetatil;
//select/ option -> useParams로 경로 데이터 받아서, 신고 요청 + 통계데이터 받아와서 일부 띄워주기 -> 통계데이터는 화장실 id로 호출
