import styles from "./CongestionItem.module.css";

const CongestionItem = ({ status }) => {
  const congestion0 = process.env.PUBLIC_URL + `/assets/c0.png`;

  const congestion1 = process.env.PUBLIC_URL + `/assets/c1.png`;

  const congestion2 = process.env.PUBLIC_URL + `/assets/c2.png`;

  const congestionB = process.env.PUBLIC_URL + `/assets/cb.png`;

  return (
    <>
      {status == 0 && <img src={congestion0} className={styles.statusImg} />}
      {status == 1 && <img src={congestion1} className={styles.statusImg} />}
      {status == 2 && <img src={congestion2} className={styles.statusImg} />}
      {status == -1 && <img src={congestionB} className={styles.statusImg} />}
    </>
  );
};

export default CongestionItem;
