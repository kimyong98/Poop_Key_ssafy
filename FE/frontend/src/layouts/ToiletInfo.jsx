import styles from "./ToiletInfo.module.css";

const ToiletInfo = () => {
  return (
    <div className={styles.footer}>
      <div className={styles.status}>
        <div className={styles.statusItem}>
          <div className={styles.statusImg1}></div>
          <div className={styles.statusText}>사용가능</div>
        </div>
        <div className={styles.statusItem}>
          <div className={styles.statusImg2}></div>
          <div className={styles.statusText}>사용중</div>
        </div>
        <div className={styles.statusItem}>
          <div className={styles.statusImg3}></div>
          <div className={styles.statusText}>고장</div>
        </div>
        <div className={styles.statusItem}>
          <div className={styles.statusImg4}></div>
          <div className={styles.statusText}>점검중</div>
        </div>
      </div>
      <div className={styles.status}>
        <div className={styles.statusItem}>
          <div className={styles.statusImg5}></div>
          <div className={styles.statusText}>원할</div>
        </div>
        <div className={styles.statusItem}>
          <div className={styles.statusImg6}></div>
          <div className={styles.statusText}>혼잡</div>
        </div>
        <div className={styles.statusItem}>
          <div className={styles.statusImg7}></div>
          <div className={styles.statusText}>포화</div>
        </div>
        <div className={styles.statusItem}>
          <div className={styles.statusImg8}></div>
          <div className={styles.statusText}>없음</div>
        </div>
      </div>
    </div>
  );
};
export default ToiletInfo;
