import styles from "./RegistItem.module.css";

const RegistItem = (props) => {
  const rows = props.height;
  const cols = props.width;

  // 기본 스타일
  const baseStyle = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,

    borderRadius: "20px",
    cursor: "pointer",
    fontWeight: "bolder",

    display: "flex",
    justifyContent: "space-around",
    alignItems: "center",
  };

  // content 값에 따라 스타일 변경
  let specificStyle;
  let imgSrc;

  switch (props.content) {
    // 빈칸
    case 0:
      specificStyle = {
        ...baseStyle,
        backgroundColor: "#C0C0C0 ",
      };
      imgSrc = {
        src: process.env.PUBLIC_URL + `/assets/x.png`,
      };
      break;

    // 화장실
    case 1:
      specificStyle = {
        ...baseStyle,
        backgroundColor: "lightgreen",
      };
      imgSrc = {
        src: process.env.PUBLIC_URL + `/assets/toilet.png`,
      };
      break;

    //입구
    case 2:
      specificStyle = {
        ...baseStyle,
        backgroundColor: "skyblue",
      };
      imgSrc = {
        src: process.env.PUBLIC_URL + `/assets/enter.png`,
      };
      break;
    default:
      specificStyle = baseStyle;
  }

  return (
    <div
      {...props}
      className={`RegistItem ${styles.iconContainer}`}
      style={specificStyle}
    >
      <img className={styles.icon} src={imgSrc.src} />
    </div>
  );
};
export default RegistItem;
