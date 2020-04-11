#include "mode_choose.h"

ModeChoose::ModeChoose()
{
    ros::param::get("~userwords_file",userwords_file);
    login();
    upload_userwords();
    mode_choose_service = m_handle.advertiseService("mode_choose",&ModeChoose::mode_choose_deal,this);

}
ModeChoose::~ModeChoose()
{
    logout();

}

void ModeChoose::login()
{
    int ret = MSP_SUCCESS;
    const char* login_params = "appid = 5d536221, work_dir = ."; // 登录参数，appid与msc库绑定,请勿随意改动
    /* 用户登录 */
    ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，均传NULL即可，第三个参数是登录参数    
    if (MSP_SUCCESS != ret)
    {
        printf("AwakeLogin failed , Error code %d.\n",ret);
    }
}

void ModeChoose::logout()
{
    MSPLogout(); //退出登录
}

int ModeChoose::upload_userwords()
{
    char* userwords = NULL;
    unsigned int len = 0;
    unsigned int read_len = 0;
    FILE* fp = NULL;
    int ret = -1;

    fp = fopen(userwords_file.c_str(), "rb");
    if (NULL == fp)
    {
        printf("\nopen [userwords.txt] failed! \n");
        goto upload_exit;
    }

    fseek(fp, 0, SEEK_END);
    len = ftell(fp); //用户词表文件大小
    fseek(fp, 0, SEEK_SET);

    userwords = (char*)malloc(len + 1);
    if (NULL == userwords)
    {
        printf("\nout of memory! \n");
        goto upload_exit;
    }

    read_len = fread((void*)userwords, 1, len, fp); //读取用户词表内容
    if (read_len != len)
    {
        printf("\nread [userwords.txt] failed!\n");
        goto upload_exit;
    }
    userwords[len] = '\0';

    MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //上传用户词表
    if (MSP_SUCCESS != ret)
    {
        printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
        goto upload_exit;
    }

upload_exit:
    if (NULL != fp)
    {
        fclose(fp);
        fp = NULL;
    }
    if (NULL != userwords)
    {
        free(userwords);
        userwords = NULL;
    }

    return ret;

}

string ModeChoose::run_iat(const char* audio_file)
{
    string iat_result;
    const char* session_begin_params = "sub = iat, domain = iat, language = zh_cn, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8";
    const char* session_id = NULL;
    char rec_result[BUFFER_SIZE] = {NULL};
    char hints[HINTS_SIZE] = {NULL}; //hints为结束本次会话的原因描述，由用户自定义
    unsigned int total_len = 0;
    int aud_stat = MSP_AUDIO_SAMPLE_CONTINUE; //音频>状态
    int ep_stat =  MSP_EP_LOOKING_FOR_SPEECH; //端点>检测
    int rec_stat = MSP_REC_STATUS_SUCCESS ;   //识别状态
    int errcode =  MSP_SUCCESS ;

    FILE* f_pcm = NULL;
    char* p_pcm = NULL;
    long pcm_count = 0;
    long pcm_size = 0;
    long read_size = 0;

    if (NULL == audio_file)
        goto iat_exit;

    f_pcm = fopen(audio_file, "rb");
    if (NULL == f_pcm)
    {
        printf("\nopen [%s] failed! \n", audio_file);
        goto iat_exit;
    }

    fseek(f_pcm, 0, SEEK_END);
    pcm_size = ftell(f_pcm); //获取音频文件大小 
    fseek(f_pcm, 0, SEEK_SET);

    p_pcm = (char *)malloc(pcm_size);
    if (NULL == p_pcm)
    {
        printf("\nout of memory! \n");
        goto iat_exit;
    }

    read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm); //读取音频文件内容
    if (read_size != pcm_size)
    {
        printf("\nread [%s] error!\n", audio_file);
        goto iat_exit;
    }

    printf("\n开始语音听写 ...\n");
    session_id = QISRSessionBegin(NULL, session_begin_params, &errcode); //听写不需要语法，第一个参数为NULL
    if (MSP_SUCCESS != errcode)
    {
        printf("\nQISRSessionBegin failed! error code:%d\n", errcode);
        goto iat_exit;
    }

    while (1)
    {
        unsigned int len = 10 * FRAME_LEN; // 每次写入200ms音频(16k，16bit)：1帧音频20ms，10帧=200ms。16k采样率的16位音频，一帧的大小为640Byte
        int ret = 0;

        if (pcm_size < 2 * len)
            len = pcm_size;
        if (len <= 0)
            break;

        aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
        if (0 == pcm_count)
            aud_stat = MSP_AUDIO_SAMPLE_FIRST;

        printf(">");
        ret = QISRAudioWrite(session_id, (const void *)&p_pcm[pcm_count], len, aud_stat, &ep_stat, &rec_stat);
        if (MSP_SUCCESS != ret)
        {
            printf("\nQISRAudioWrite failed! error code:%d\n", ret);
            goto iat_exit;
        }

        pcm_count += (long)len;
        pcm_size  -= (long)len;

        if (MSP_REC_STATUS_SUCCESS == rec_stat) //已经有部分听写结果
        {
            const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
            if (MSP_SUCCESS != errcode)
            {
                printf("\nQISRGetResult failed! error code: %d\n", errcode);
                goto iat_exit;
            }
            if (NULL != rslt)
            {
                unsigned int rslt_len = strlen(rslt);
                total_len += rslt_len;
                if (total_len >= BUFFER_SIZE)
                {
                    printf("\nno enough buffer for rec_result !\n");
                    goto iat_exit;
                }
                strncat(rec_result, rslt, rslt_len);
            }
        }

        if (MSP_EP_AFTER_SPEECH == ep_stat)
            break;
        usleep(20*1000); //模拟人说话时间间隙。200ms对应10帧的音频
    }
    errcode = QISRAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_stat, &rec_stat);
    if (MSP_SUCCESS != errcode)
    {
        printf("\nQISRAudioWrite failed! error code:%d \n", errcode);
        goto iat_exit;
    }

    while (MSP_REC_STATUS_COMPLETE != rec_stat)
    {
        const char *rslt = QISRGetResult(session_id, &rec_stat, 0, &errcode);
        if (MSP_SUCCESS != errcode)
        {
            printf("\nQISRGetResult failed, error code: %d\n", errcode);
            goto iat_exit;
        }
        if (NULL != rslt)
        {
            unsigned int rslt_len = strlen(rslt);
            total_len += rslt_len;
            if (total_len >= BUFFER_SIZE)
            {
                printf("\nno enough buffer for rec_result !\n");
                goto iat_exit;
            }
            strncat(rec_result, rslt, rslt_len);
        }
        usleep(15*1000); //防止频繁占用CPU
    }
    printf("=============================================================\n");
    printf("%s\n",rec_result);
    printf("=============================================================\n");
    iat_result = rec_result;

iat_exit:
    if (NULL != f_pcm)
    {
        fclose(f_pcm);
        f_pcm = NULL;
    }
    if (NULL != p_pcm)
    {       free(p_pcm);
        p_pcm = NULL;
    }

    QISRSessionEnd(session_id, hints);

    return iat_result;	
}

bool ModeChoose::mode_choose_deal(voice_msgs::mode_choose::Request &req,voice_msgs::mode_choose::Response &res)
{
    string iat_ret;
    iat_ret = run_iat(req.audio_file.data());		
    if(iat_ret.find("闲聊模式") != string::npos){
        res.mode = "闲聊模式";
    }
    else if(iat_ret.find("控制模式") != string::npos){
        res.mode = "控制模式";
    }
    else if(iat_ret.find("导航模式") != string::npos){
        res.mode = "导航模式";
    }
    else if(iat_ret.find("休息") != string::npos){
        res.mode = "休眠";
    }
    else{
        res.mode = "模式未识别";
    }

    return true;
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"mode_choose");
    ModeChoose  modechoose;

    ros::spin();
}








