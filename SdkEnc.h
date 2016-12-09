//#ifndef	_SDKENC_H_
#define	_SDKENC_H_

// 永野追加
extern "C"
#include "stdafx.h"
//#include "common_utils.h"
//#include "cmd_options.h"
#include "mfxvideo++2.h"
#include "mfxdefs.h"
#include "mfxstructures.h"

#include	<vector>
#include	<list>

#include <opencv2\Common_OpenCV.hpp>


// option---------------------------------------------------------------------------------------------------------
#define OPTION_IMPL             0x001
#define OPTION_GEOMETRY         0x002
#define OPTION_BITRATE          0x004
#define OPTION_FRAMERATE        0x008
#define OPTION_MEASURE_LATENCY  0x010

#define OPTIONS_DECODE \
	(OPTION_IMPL)

#define OPTIONS_ENCODE \
	(OPTION_IMPL | OPTION_GEOMETRY | OPTION_BITRATE | OPTION_FRAMERATE)

#define OPTIONS_VPP \
	(OPTION_IMPL | OPTION_GEOMETRY)

#define OPTIONS_TRANSCODE \
	(OPTION_IMPL | OPTION_BITRATE | OPTION_FRAMERATE)

#define MSDK_MAX_PATH 280

struct CmdOptionsCtx {
	// bitmask of the options accepted by the program
	unsigned int options;
	// program name, if not set will be corrected to argv[0]
	const char* program;
	// function to print program usage, can be NULL
	void (*usage)(CmdOptionsCtx* ctx);
};

struct CmdOptionsValues {
	mfxIMPL impl; // OPTION_IMPL

	char SourceName[MSDK_MAX_PATH]; // OPTION_FSOURCE
	char SinkName[MSDK_MAX_PATH];   // OPTION_FSINK

	mfxU16 Width; // OPTION_GEOMETRY
	mfxU16 Height;

	mfxU16 Bitrate; // OPTION_BITRATE

	mfxU16 FrameRateN; // OPTION_FRAMERATE
	mfxU16 FrameRateD;

	bool MeasureLatency; // OPTION_MEASURE_LATENCY
};

struct CmdOptions {
	CmdOptionsCtx ctx;
	CmdOptionsValues values;
};
// utils---------------------------------------------------------------------------------------------------------
#if defined(_WIN32) || defined(_WIN64)
#include "bits/windows_defs.h"
#elif defined(__linux__)
#include "bits/linux_defs.h"
#endif
// =================================================================
// Helper macro definitions...
#define MSDK_PRINT_RET_MSG(ERR)         {PrintErrString(ERR, __FILE__, __LINE__);}
#define MSDK_CHECK_RESULT(P, X, ERR)    {if ((X) > (P)) {MSDK_PRINT_RET_MSG(ERR); return ERR;}}
#define MSDK_CHECK_POINTER(P, ERR)      {if (!(P)) {MSDK_PRINT_RET_MSG(ERR); return ERR;}}
#define MSDK_CHECK_ERROR(P, X, ERR)     {if ((X) == (P)) {MSDK_PRINT_RET_MSG(ERR); return ERR;}}
#define MSDK_IGNORE_MFX_STS(P, X)       {if ((X) == (P)) {P = MFX_ERR_NONE;}}
#define MSDK_BREAK_ON_ERROR(P)          {if (MFX_ERR_NONE != (P)) break;}
#define MSDK_SAFE_DELETE_ARRAY(P)       {if (P) {delete[] P; P = NULL;}}
#define MSDK_ALIGN32(X)                 (((mfxU32)((X)+31)) & (~ (mfxU32)31))
#define MSDK_ALIGN16(value)             (((value + 15) >> 4) << 4)
#define MSDK_SAFE_RELEASE(X)            {if (X) { X->Release(); X = NULL; }}
#define MSDK_MAX(A, B)                  (((A) > (B)) ? (A) : (B))

// Usage of the following two macros are only required for certain Windows DirectX11 use cases
#define WILL_READ  0x1000
#define WILL_WRITE 0x2000

// =================================================================
// Intel Media SDK memory allocator entrypoints....
// Implementation of this functions is OS/Memory type specific.
mfxStatus simple_alloc(mfxHDL pthis, mfxFrameAllocRequest* request, mfxFrameAllocResponse* response);
mfxStatus simple_lock(mfxHDL pthis, mfxMemId mid, mfxFrameData* ptr);
mfxStatus simple_unlock(mfxHDL pthis, mfxMemId mid, mfxFrameData* ptr);
mfxStatus simple_gethdl(mfxHDL pthis, mfxMemId mid, mfxHDL* handle);
mfxStatus simple_free(mfxHDL pthis, mfxFrameAllocResponse* response);

// For use with asynchronous task management
typedef struct {
	mfxBitstream mfxBS;
	mfxSyncPoint syncp;
} Task;


//Takagi
static struct UV
{
	std::vector<float> fU;
	std::vector<float> fV;
};

// 永野---------------------------------------------------------------------------------------------------------
class	CSdkEnc
{
	//MFXVideoSession m_session;

public:
	CSdkEnc();
	~CSdkEnc();
	BOOL		MainEnc();
	//void		ParseOptions(CmdOptions* cmd_options);
	void		ParseOptions(CmdOptions* cmd_options,cv::Mat InpMat);
	mfxStatus	ReadPlaneData(mfxU16 w, mfxU16 h, mfxU8* buf, mfxU8* ptr, mfxU16 pitch, mfxU16 offset, FILE* fSource);
	//mfxStatus	LoadRawFrame(mfxFrameSurface1* pSurface, FILE* fSource);
	mfxStatus   LoadRawFrame(mfxFrameSurface1* pSurface,cv::Mat InpMat, UV fUV);
	mfxStatus	LoadRawRGBFrame(mfxFrameSurface1* pSurface, FILE* fSource);
	void		PrintErrString(int err,const char* filestr,int line);
	mfxStatus	WriteBitStreamFrame(mfxBitstream* pMfxBitstream, FILE* fSink);		// Write bit stream data for frame to file
	mfxStatus	ReadBitStreamData(mfxBitstream* pBS, FILE* fSource);					// Read bit stream data from file. Stream is read as large chunks (= many frames)
	mfxStatus	WriteSection(mfxU8* plane, mfxU16 factor, mfxU16 chunksize, mfxFrameInfo* pInfo, mfxFrameData* pData, mfxU32 i, mfxU32 j, FILE* fSink);
	mfxStatus	WriteRawFrame(mfxFrameSurface1* pSurface, FILE* fSink);

	void ClearYUVSurfaceSysMem(mfxFrameSurface1* pSfc, mfxU16 width, mfxU16 height);
	void ClearYUVSurfaceVMem(mfxMemId memId);
	void ClearRGBSurfaceVMem(mfxMemId memId);
	
	// Get free raw frame surface
	int GetFreeSurfaceIndex(mfxFrameSurface1** pSurfacesPool, mfxU16 nPoolSize);
	int GetFreeTaskIndex(Task* pTaskPool, mfxU16 nPoolSize);
	mfxStatus Initialize(mfxIMPL impl, mfxVersion ver, MFXVideoSession* pSession, mfxFrameAllocator* pmfxAllocator, bool bCreateSharedHandles = false);	// Initialize Intel Media SDK Session, device/display and memory manager
	void Release();													// Release resources (device/display)
	char mfxFrameTypeString(mfxU16 FrameType);						// Convert frame type to string
	void mfxGetTime(mfxTime* timestamp);
	double TimeDiffMsec(mfxTime tfinish, mfxTime tstart);			//void mfxInitTime();  might need this for Windows

	//void PrintErrString(int err,const char* filestr,int line);
	//Encoder 関連
	cv::Mat				RGB2YUV422( cv::Mat RGBMat );
	//cv::Mat				RGB2YUV420(cv::Mat RGBMat);
	void				RGB2YUV420(cv::Mat RGBMat,cv::Mat &YUVMat, UV &fUV);
	BOOL				PreparaStage1(cv::Mat YUVMat, CmdOptions& options, MFXVideoSession&	cSession ,bool& bEnableOutput);
	mfxVideoParam		EncodePrmInit( MFXVideoSession& cSession, CmdOptions options );
	BOOL				SecureMem( mfxVideoParam mfxEncParams ,MFXVideoENCODE mfxENC, mfxU8* surfaceBuffers ,mfxU32 surfaceSize, mfxU16 nEncSurfNum , mfxFrameSurface1** pEncSurfaces);
	//mfxFrameSurface1**	SecureMem( mfxVideoParam mfxEncParams ,MFXVideoENCODE mfxENC);
};

inline	CSdkEnc::CSdkEnc()
{
}

inline	CSdkEnc::~CSdkEnc()
{
}

inline void CSdkEnc::PrintErrString(int err,const char* filestr,int line)
{
	CString	strOutput;
	switch (err) {
	case   0:
		strOutput.Format("\n No error.\n");
		::OutputDebugStringA(strOutput);
		break;
	case  -1:
		 strOutput.Format("\n Unknown error: %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -2:
		 strOutput.Format("\n Null pointer.  Check filename/path + permissions? %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -3:
		strOutput.Format("\n Unsupported feature/library load error. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -4:
		strOutput.Format("\n Could not allocate memory. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -5:
		strOutput.Format("\n Insufficient IO buffers. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -6:
		strOutput.Format("\n Invalid handle. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -7:
		strOutput.Format("\n Memory lock failure. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -8:
		strOutput.Format("\n Function called before initialization. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case  -9:
		strOutput.Format("\n Specified object not found. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -10:
		strOutput.Format("\n More input data expected. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -11:
		strOutput.Format("\n More output surfaces expected. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -12:
		strOutput.Format("\n Operation aborted. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -13:
		strOutput.Format("\n HW device lost. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -14:
		strOutput.Format("\n Incompatible video parameters. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -15:
		strOutput.Format("\n Invalid video parameters. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -16:
		strOutput.Format("\n Undefined behavior. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -17:
		strOutput.Format("\n Device operation failure. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -18:
		strOutput.Format("\n More bitstream data expected. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -19:
		strOutput.Format("\n Incompatible audio parameters. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	case -20:
		strOutput.Format("\n Invalid audio parameters. %s %d\n",filestr,line);
		::OutputDebugStringA(strOutput);
		break;
	default:
		strOutput.Format("\nError code %d,\t%s\t%d\n\n", err, filestr, line);
		::OutputDebugStringA(strOutput);
	}
}

inline	mfxStatus CSdkEnc::ReadPlaneData(mfxU16 w, mfxU16 h, mfxU8* buf, mfxU8* ptr,
						mfxU16 pitch, mfxU16 offset, FILE* fSource)
{
	mfxU32 nBytesRead;
	for (mfxU16 i = 0; i < h; i++) {
		nBytesRead = (mfxU32) fread(buf, 1, w, fSource);
		if (w != nBytesRead)
			return MFX_ERR_MORE_DATA;
		for (mfxU16 j = 0; j < w; j++)
			ptr[i * pitch + j * 2 + offset] = buf[j];
	}
	return MFX_ERR_NONE;
}

//inline	mfxStatus CSdkEnc::LoadRawFrame(mfxFrameSurface1* pSurface, FILE* fSource)
inline	mfxStatus CSdkEnc::LoadRawFrame(mfxFrameSurface1* pSurface/*, FILE* fSource*/,cv::Mat InpMat, UV fUV)

{
	/*
	if (!fSource) {
		// Simulate instantaneous access to 1000 "empty" frames.
		static int frameCount = 0;
		if (1000 == frameCount++)
			return MFX_ERR_MORE_DATA;
		else
			return MFX_ERR_NONE;
	}
	*/
	mfxStatus sts = MFX_ERR_NONE;
	//mfxU32 nBytesRead;
//	mfxU16 w, h, i, pitch;
	mfxU16 w, h, pitch;
	mfxU8* ptr;
	mfxFrameInfo* pInfo = &pSurface->Info;
	mfxFrameData* pData = &pSurface->Data;

	if (pInfo->CropH > 0 && pInfo->CropW > 0) {
		w = pInfo->CropW;
		h = pInfo->CropH;
	} else {
		w = pInfo->Width;
		h = pInfo->Height;
	}

	pitch = pData->Pitch;
	ptr = pData->Y + pInfo->CropX + pInfo->CropY * pData->Pitch;
	//::OutputDebugString("Start Reading RawData\n");
	// read luminance plane

	//LPBYTE	ImageData	=	InpMat.data ;
	//Read Pointer
	cv::Vec3b*	YUVScr;
	cv::Vec3b	YUV;
	//::OutputDebugString("Reading Y channel\n");
	int iCheck;
	for (int iLoopH = 0; iLoopH < h; iLoopH++) {
		//nBytesRead = (mfxU32) fread(ptr + i * pitch, 1, w, fSource);
		YUVScr = InpMat.ptr< cv::Vec3b >(iLoopH );
		for(int iLoopW=0;iLoopW< w ;iLoopW++)
		{
			//ポインタの移動
			//ptr = pData->Y + i * pitch + nLoopX;
			//値の変更
			//*ptr = (mfxU8)ImageData[nLoopX*3 + 0];
			YUV = YUVScr[ iLoopW ];
			ptr[ iLoopH * pitch + iLoopW ] = YUV[0];	//Y
			//iCheck++;
		}
		//ImageData	+=	InpMat.step;

		//if ( w != iCheck )
		//	return MFX_ERR_MORE_DATA;
	}

	mfxU8 buf[2048];        // maximum supported chroma width for nv12
	w /= 2;
	h /= 2;
	ptr = pData->UV + pInfo->CropX + (pInfo->CropY / 2) * pitch;
	if (w > 2048)
		return MFX_ERR_UNSUPPORTED;

	//Uチャンネル読み込み
	//mfxU32 nBytesRead;
	//ImageData	=	InpMat.data ;
	//::OutputDebugString("Reading U and V channel\n");
	//iCheck = 1;
	//cv::Vec3b*	YUVScr2;
	//cv::Vec3b	YUV2;
	UINT iUVNum = 0;
	float fU,fV;
	for (mfxU16 iLoopH = 0; iLoopH < h; iLoopH++) {
		//YUVScr2 = InpMat.ptr< cv::Vec3b >(iLoopH );
		/*nBytesRead = (mfxU32) fread(buf, 1, w, fSource);*/
		for (mfxU16 iLoopW = 0; iLoopW < w; iLoopW++)
		{
			fU = fUV.fU[ iUVNum ];
			fV = fUV.fV[ iUVNum ];
			ptr[  iLoopH * pitch + iLoopW * 2 + 0 ] = fU;	//U
			ptr[  iLoopH * pitch + iLoopW * 2 + 1 ] = fV;	//V
			iUVNum++;
			//YUV2 = YUVScr2[ iLoopW ];
			//ptr[  iLoopH * pitch + iLoopW * 2 + 0 ] = YUV2[1];	//U
			//ptr[  iLoopH * pitch + iLoopW * 2 + 1 ] = YUV2[2];	//V
			
			//ptr[ iLoopH * pitch + iLoopW * 2 + 0 ] = (mfxU8)ImageData[ iLoopW * 3 + 1];	//U
			//ptr[ iLoopH * pitch + iLoopW * 2 + 1 ] = (mfxU8)ImageData[ iLoopW * 3 + 2];	//V
		}
		//ImageData	+=	InpMat.step;

		//if ( w != iCheck )
		//	return MFX_ERR_MORE_DATA;
	}
	/*    for (i = 0; i < h; i++) {
		nBytesRead = (mfxU32) fread(ptr + i * pitch, 1, w, fSource);
		if (w != nBytesRead)
			return MFX_ERR_MORE_DATA;
	}*/

	/*
	return MFX_ERR_NONE;

	// load U
	sts = ReadPlaneData(w, h, buf, ptr, pitch, 0, fSource);
	if (MFX_ERR_NONE != sts)
		return sts;
	// load V
	ReadPlaneData(w, h, buf, ptr, pitch, 1, fSource);
	if (MFX_ERR_NONE != sts)
		return sts;
	*/
	return MFX_ERR_NONE;
}

inline	mfxStatus CSdkEnc::LoadRawRGBFrame(mfxFrameSurface1* pSurface, FILE* fSource)
{
	if (!fSource) {
		// Simulate instantaneous access to 1000 "empty" frames.
		static int frameCount = 0;
		if (1000 == frameCount++)
			return MFX_ERR_MORE_DATA;
		else
			return MFX_ERR_NONE;
	}

	size_t nBytesRead;
	mfxU16 w, h;
	mfxFrameInfo* pInfo = &pSurface->Info;

	if (pInfo->CropH > 0 && pInfo->CropW > 0) {
		w = pInfo->CropW;
		h = pInfo->CropH;
	} else {
		w = pInfo->Width;
		h = pInfo->Height;
	}

	for (mfxU16 i = 0; i < h; i++) {
		nBytesRead = fread(pSurface->Data.B + i * pSurface->Data.Pitch,
						   1, w * 4, fSource);
		if ((size_t)(w * 4) != nBytesRead)
			return MFX_ERR_MORE_DATA;
	}

	return MFX_ERR_NONE;
}

inline	mfxStatus CSdkEnc::WriteBitStreamFrame(mfxBitstream* pMfxBitstream, FILE* fSink)
{
	mfxU32 nBytesWritten =
		(mfxU32) fwrite(pMfxBitstream->Data + pMfxBitstream->DataOffset, 1,
						pMfxBitstream->DataLength, fSink);
	if (nBytesWritten != pMfxBitstream->DataLength)
		return MFX_ERR_UNDEFINED_BEHAVIOR;

	pMfxBitstream->DataLength = 0;

	return MFX_ERR_NONE;
}

inline	mfxStatus CSdkEnc::ReadBitStreamData(mfxBitstream* pBS, FILE* fSource)
{
	memmove(pBS->Data, pBS->Data + pBS->DataOffset, pBS->DataLength);
	pBS->DataOffset = 0;

	mfxU32 nBytesRead = (mfxU32) fread(pBS->Data + pBS->DataLength, 1,
									   pBS->MaxLength - pBS->DataLength,
									   fSource);

	if (0 == nBytesRead)
		return MFX_ERR_MORE_DATA;

	pBS->DataLength += nBytesRead;

	return MFX_ERR_NONE;
}

inline	mfxStatus CSdkEnc::WriteSection(mfxU8* plane, mfxU16 factor, mfxU16 chunksize,
					   mfxFrameInfo* pInfo, mfxFrameData* pData, mfxU32 i,
					   mfxU32 j, FILE* fSink)
{
	if (chunksize !=
		fwrite(plane +
			   (pInfo->CropY * pData->Pitch / factor + pInfo->CropX) +
			   i * pData->Pitch + j, 1, chunksize, fSink))
		return MFX_ERR_UNDEFINED_BEHAVIOR;
	return MFX_ERR_NONE;
}

inline	mfxStatus CSdkEnc::WriteRawFrame(mfxFrameSurface1* pSurface, FILE* fSink)
{
	mfxFrameInfo* pInfo = &pSurface->Info;
	mfxFrameData* pData = &pSurface->Data;
	mfxU32 i, j, h, w;
	mfxStatus sts = MFX_ERR_NONE;

	for (i = 0; i < pInfo->CropH; i++)
		sts =
			WriteSection(pData->Y, 1, pInfo->CropW, pInfo, pData, i, 0,
						 fSink);

	h = pInfo->CropH / 2;
	w = pInfo->CropW;
	for (i = 0; i < h; i++)
		for (j = 0; j < w; j += 2)
			sts =
				WriteSection(pData->UV, 2, 1, pInfo, pData, i, j,
							 fSink);
	for (i = 0; i < h; i++)
		for (j = 1; j < w; j += 2)
			sts =
				WriteSection(pData->UV, 2, 1, pInfo, pData, i, j,
							 fSink);

	return sts;
}

inline	int CSdkEnc::GetFreeTaskIndex(Task* pTaskPool, mfxU16 nPoolSize)
{
	if (pTaskPool)
		for (int i = 0; i < nPoolSize; i++)
			if (!pTaskPool[i].syncp)
				return i;
	return MFX_ERR_NOT_FOUND;
}

inline	mfxStatus CSdkEnc::Initialize(mfxIMPL impl, mfxVersion ver, MFXVideoSession* pSession, mfxFrameAllocator* pmfxAllocator, bool bCreateSharedHandles)
{
	mfxStatus sts = MFX_ERR_NONE;

#ifdef DX11_D3D
	impl |= MFX_IMPL_VIA_D3D11;
#endif

	// Initialize Intel Media SDK Session
	sts = pSession->Init(impl, &ver);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

#if defined(DX9_D3D) || defined(DX11_D3D)
	// If mfxFrameAllocator is provided it means we need to setup DirectX device and memory allocator
	if (pmfxAllocator) {
		// Create DirectX device context
		mfxHDL deviceHandle;
		sts = CreateHWDevice(*pSession, &deviceHandle, NULL, bCreateSharedHandles);
		MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

		// Provide device manager to Media SDK
		sts = pSession->SetHandle(DEVICE_MGR_TYPE, deviceHandle);
		MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

		pmfxAllocator->pthis  = *pSession; // We use Media SDK session ID as the allocation identifier
		pmfxAllocator->Alloc  = simple_alloc;
		pmfxAllocator->Free   = simple_free;
		pmfxAllocator->Lock   = simple_lock;
		pmfxAllocator->Unlock = simple_unlock;
		pmfxAllocator->GetHDL = simple_gethdl;

		// Since we are using video memory we must provide Media SDK with an external allocator
		sts = pSession->SetFrameAllocator(pmfxAllocator);
		MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
	}
#endif

	return sts;
}

inline	void CSdkEnc::Release()
{
#if defined(DX9_D3D) || defined(DX11_D3D)
	CleanupHWDevice();
#endif
}

inline	char CSdkEnc::mfxFrameTypeString(mfxU16 FrameType)
{
	mfxU8 FrameTmp = FrameType & 0xF;
	char FrameTypeOut;
	switch (FrameTmp) {
	case MFX_FRAMETYPE_I:
		FrameTypeOut = 'I';
		break;
	case MFX_FRAMETYPE_P:
		FrameTypeOut = 'P';
		break;
	case MFX_FRAMETYPE_B:
		FrameTypeOut = 'B';
		break;
	default:
		FrameTypeOut = '*';
	}
	return FrameTypeOut;
}

inline	void CSdkEnc::mfxGetTime(mfxTime* timestamp)
{
	QueryPerformanceCounter(timestamp);
}

inline	double CSdkEnc::TimeDiffMsec(mfxTime tfinish, mfxTime tstart)
{
	static LARGE_INTEGER tFreq = { 0 };

	if (!tFreq.QuadPart) QueryPerformanceFrequency(&tFreq);

	double freq = (double)tFreq.QuadPart;
	return 1000.0 * ((double)tfinish.QuadPart - (double)tstart.QuadPart) / freq;
}

inline	void CSdkEnc::ClearYUVSurfaceSysMem(mfxFrameSurface1* pSfc, mfxU16 width, mfxU16 height)
{
	// In case simulating direct access to frames we initialize the allocated surfaces with default pattern
	memset(pSfc->Data.Y, 100, width * height);  // Y plane
	memset(pSfc->Data.U, 50, (width * height)/2);  // UV plane
}

inline	void CSdkEnc::ClearYUVSurfaceVMem(mfxMemId memId)
{
#if defined(DX9_D3D) || defined(DX11_D3D)
	ClearYUVSurfaceD3D(memId);
#endif
}

inline	void CSdkEnc::ClearRGBSurfaceVMem(mfxMemId memId)
{
#if defined(DX9_D3D) || defined(DX11_D3D)
	ClearRGBSurfaceD3D(memId);
#endif
}


// Get free raw frame surface
inline	int CSdkEnc::GetFreeSurfaceIndex(mfxFrameSurface1** pSurfacesPool, mfxU16 nPoolSize)
{
	if (pSurfacesPool)
		for (mfxU16 i = 0; i < nPoolSize; i++)
			if (0 == pSurfacesPool[i]->Data.Locked)
				return i;
	return MFX_ERR_NOT_FOUND;
}

inline void	CSdkEnc::ParseOptions(CmdOptions* cmd_options,cv::Mat InpMat)
{
	ASSERT( InpMat.data );
	int iBitRate = 0;
	//ソフトウェアエンコード	ハードウェアエンコード：MFX_IMPL_HARDWARE 自動選択：MFX_IMPL_AUTO
	cmd_options->values.impl =MFX_IMPL_AUTO;
	//入力ファイルから画像サイズを入力 仮にcv::Matを利用
//	if( InpMat.data )
//	{
//		cmd_options->values.Width = (mfxU16)InpMat.cols;;
//		cmd_options->values.Height = (mfxU16)InpMat.rows;
//	}
//	else	::OutputDebugStringA( "line(215) Input file empth" );
	//ビットレートの設定
	cmd_options->values.Bitrate = (mfxU16)5000;
	//フレームレート設定
	cmd_options->values.FrameRateN = (mfxU16)30;
	cmd_options->values.FrameRateD = (mfxU16)1;
	//待ち時間を計測するかどうか？よくわからないのでFalse
	cmd_options->values.MeasureLatency = true;
	//画像のサイズ
//	cmd_options->values.Width	= (mfxU16)InpMat.cols;
//	cmd_options->values.Height	= (mfxU16)InpMat.rows;
	cmd_options->values.Width	= (mfxU16)1280;
	cmd_options->values.Height	= (mfxU16)960;

	//cmd_options->values.Width = (mfxU16)1920;
	//cmd_options->values.Height = (mfxU16)1080;
	//cmd_options->values.Bitrate = (mfxU16)5000;
	//cmd_options->values.FrameRateN = (mfxU16)30;
	//cmd_options->values.FrameRateD = (mfxU16)1;
		//入力ファイル名
	//
	//strcpy_s(cmd_options->values.SourceName, "offroad.yuv" );		//
	//cmd_options->values.SourceName = cFile;			//
	//出力ファイル名									//

	//出力ファイル名
	strcpy_s(cmd_options->values.SinkName,"kuma.264" );	
}


inline	void	CSdkEnc::RGB2YUV420(cv::Mat RGBMat,cv::Mat &YUVMat, UV &fUV)
{
	//RGB画像→YUV画像への変換
	YUVMat = cv::Mat( cv::Size( RGBMat.cols , RGBMat.rows),CV_8UC3 );
	UCHAR	Blue1,Red1,Green1,Blue2,Red2,Green2;
	cv::Vec3b	*RawScr1,*RawScr2,*YUVScr1,*YUVScr2;
	cv::Vec3b	RGB3b11,RGB3b12,YUV3b11,YUV3b12;
	cv::Vec3b	RGB3b21,RGB3b22,YUV3b21,YUV3b22;

	for(int iLoopH = 0;iLoopH < RGBMat.rows/2 ;iLoopH += 2)
	{
		//pixelブロック取得
		RawScr1	=	RGBMat.ptr< cv::Vec3b >( iLoopH );
		RawScr2	=	RGBMat.ptr< cv::Vec3b >( iLoopH + 1);
		YUVScr1	=	YUVMat.ptr< cv::Vec3b >( iLoopH );
		YUVScr2	=	YUVMat.ptr< cv::Vec3b >( iLoopH + 1);
		//Y
		for (int iLoopW = 0; iLoopW < RGBMat.cols/2; iLoopW+=2)
		{
			//RGB値取得
			RGB3b11	=	RawScr1[ iLoopW ];
			RGB3b12	=	RawScr1[ iLoopW + 1 ];
			RGB3b21	=	RawScr2[ iLoopW ];
			RGB3b22	=	RawScr2[ iLoopW + 1 ];
			//Yチャンネル入力
			YUV3b11[ 0 ] =  0.257 * RGB3b11[ 2 ]	+	0.504 * RGB3b11[ 1 ] 	+	0.098 * RGB3b11[ 0 ]	+	16;
			YUV3b12[ 0 ] =  0.257 * RGB3b12[ 2 ]	+	0.504 * RGB3b12[ 1 ] 	+	0.098 * RGB3b12[ 0 ]	+	16;
			YUV3b21[ 0 ] =  0.257 * RGB3b21[ 2 ]	+	0.504 * RGB3b21[ 1 ] 	+	0.098 * RGB3b21[ 0 ]	+	16;
			YUV3b22[ 0 ] =  0.257 * RGB3b22[ 2 ]	+	0.504 * RGB3b22[ 1 ] 	+	0.098 * RGB3b22[ 0 ]	+	16;
			//出力
			YUV3b11	=	YUVScr1[ iLoopW ];
			YUV3b12	=	YUVScr1[ iLoopW + 1 ];
			YUV3b21	=	YUVScr2[ iLoopW ];
			YUV3b22	=	YUVScr2[ iLoopW + 1 ];
		}
	}
	//UVチャンネルを入力（とりあえず実装）
	
	//float fU[ 640 * 480 ],fV[ 640 * 480 ];
	int iUVNum = 0;
	for(int iLoopH = 0;iLoopH < RGBMat.rows ;iLoopH += 2)
	{
		for (int iLoopW = 0; iLoopW < RGBMat.cols; iLoopW+=2,iUVNum++)
		{
			//RGB値取得
			RGB3b21	=	RawScr2[ iLoopW ];
			//UVチャンネル入力
			fUV.fU.push_back(	 (  0.439 * RGB3b21[ 2 ]	-	0.368 * RGB3b21[ 1 ]	-	0.071 * RGB3b21[ 0 ] )	+	128	);
			fUV.fV.push_back(	 ( -0.148 * RGB3b21[ 2 ]	-	0.291 * RGB3b21[ 1 ]	+	0.439 * RGB3b21[ 0 ] )  +	128	);
			
			//fUV.fU[ iUVNum ] =	 (  0.439 * RGB3b21[ 2 ]	-	0.368 * RGB3b21[ 1 ]	-	0.071 * RGB3b21[ 0 ] )	+	128	;
			//fUV.fV[ iUVNum ] =	 ( -0.148 * RGB3b21[ 2 ]	-	0.291 * RGB3b21[ 1 ]	+	0.439 * RGB3b21[ 0 ] )  +	128	;
		}
	}
}


inline	cv::Mat	CSdkEnc::RGB2YUV422(cv::Mat RGBMat)
{
	//RGB画像→YUV画像への変換
	cv::Mat YUVMat( cv::Size( RGBMat.cols , RGBMat.rows),CV_8UC3 );
	UCHAR	Blue1,Red1,Green1,Blue2,Red2,Green2;
	cv::Vec3b	*RawScr,*YUVScr;
	cv::Vec3b	RGB3b1,RGB3b2,YUV3b1,YUV3b2;
	for(int iLoopH = 0;iLoopH < RGBMat.rows ;iLoopH++)
	{
		RawScr	=	RGBMat.ptr< cv::Vec3b >( iLoopH );
		YUVScr	=	YUVMat.ptr< cv::Vec3b >( iLoopH );
		for (int iLoopW = 0; iLoopW < RGBMat.cols/2; iLoopW+=2)
		{
			//RGB値取得
			RGB3b1	=	RawScr[ iLoopW ];
			RGB3b2	=	RawScr[ iLoopW + 1 ];
			Blue1	=	RGB3b1[ 0 ];
			Green1	=	RGB3b1[ 1 ];
			Red1	=	RGB3b1[ 2 ];
			Blue2	=	RGB3b2[ 0 ];
			Green2	=	RGB3b2[ 1 ];
			Red2	=	RGB3b2[ 2 ];
			//YUV計算
			//1pixel目
			YUV3b1[ 0 ] =	 0.257 * Red1	+	0.504 * Green1	+	0.098 * Blue1	+	16	;
			YUV3b1[ 1 ] =	 ( ( 0.439 * Red1	-	0.368 * Green1	-	0.071 * Blue1 )	+	( 0.439 * Red2	-	0.368 * Green2	-	0.071 * Blue2 ) ) / 2	+	128	;
			YUV3b1[ 2 ] =	 ( ( -0.148 * Red1	-	0.291 * Green1	+	0.439 * Blue1 ) + ( -0.148 * Red2	-	0.291 * Green2	+	0.439 * Blue2 ) ) / 2	+	128	;
			//2pixel目
			YUV3b2[ 0 ] =	 0.257 * Red2	+	0.504 * Green2	+	0.098 * Blue2	+	16	;
			YUV3b2[ 1 ] = YUV3b1[ 1 ];
			YUV3b2[ 2 ] = YUV3b1[ 2 ];

			YUVScr[ iLoopW ]		=	YUV3b1;
			YUVScr[ iLoopW + 1 ]	=	YUV3b2;
		}
	}
	return YUVMat;


	////RGB画像→YUV画像への変換
	//cv::Mat YUVMat( cv::Size( RGBMat.cols , RGBMat.rows),CV_8UC3 );
	//UCHAR	Blue,Red,Green;
	//cv::Vec3b	*RawScr,*YUVScr;
	//cv::Vec3b	RGB3b,YUV3b;
	//for(int iLoopH = 0;iLoopH < RGBMat.rows;iLoopH++)
	//{
	//	RawScr	=	RGBMat.ptr< cv::Vec3b >( iLoopH );
	//	YUVScr	=	YUVMat.ptr< cv::Vec3b >( iLoopH );
	//	for (int iLoopW = 0; iLoopW < RGBMat.cols; iLoopW++)
	//	{
	//		//RGB値取得
	//		RGB3b	=	RawScr[ iLoopW ];
	//		Blue	=	RGB3b[ 0 ];
	//		Green	=	RGB3b[ 1 ];
	//		Red		=	RGB3b[ 2 ];
	//		//YUV計算
	//		YUV3b[ 0 ] =	 0.257 * Red	+	0.504 * Green	+	0.098 * Blue	+	16	;
	//		YUV3b[ 1 ] =	 0.439 * Red	-	0.368 * Green	-	0.071 * Blue	+	128	;
	//		YUV3b[ 2 ] =	-0.148 * Red	-	0.291 * Green	+	0.439 * Blue	+	128	;
	//		YUVScr[ iLoopW ]	=	YUV3b;
	//	}
	//}
	//return YUVMat;
}

inline BOOL	CSdkEnc::PreparaStage1(cv::Mat YUVMat, CmdOptions& options, MFXVideoSession&	cSession ,bool& bEnableOutput)
{
	//SDK Session Initialize
	CString strOutput;
	mfxStatus sts = MFX_ERR_NONE;
	//CmdOptions options;

	memset(&options, 0, sizeof(CmdOptions));
	options.ctx.options = OPTIONS_ENCODE;
	//options.ctx.usage = usage;							// よくわからん(使用方法の表示)?
	// Set default values:
	options.values.impl = MFX_IMPL_AUTO_ANY;

	// here we parse options
	ParseOptions(&options,YUVMat);
	
		if (!options.values.Width || !options.values.Height) {
		strOutput.Format("error: input video geometry not set (mandatory)\n");
		::OutputDebugString(strOutput);
		return FALSE;
	}
	if (!options.values.Bitrate) {
		strOutput.Format("error: bitrate not set (mandatory)\n");
		::OutputDebugString(strOutput);
		return FALSE;
	}
	if (!options.values.FrameRateN || !options.values.FrameRateD) {
		strOutput.Format("error: framerate not set (mandatory)\n");
		::OutputDebugString(strOutput);
		return FALSE;
	}
	bEnableOutput = (options.values.SinkName[0] != '\0');

	// Create output elementary stream (ES) H.264 file	//
	//FILE* fSink = NULL;
	//if (bEnableOutput) {
	//	MSDK_FOPEN(fSink, options.values.SinkName, "wb");
	//	MSDK_CHECK_POINTER(fSink, MFX_ERR_NULL_PTR);
	//}

	// Initialize Intel Media SDK session
	// - MFX_IMPL_AUTO_ANY selects HW acceleration if available (on any adapter)
	// - Version 1.0 is selected for greatest backwards compatibility.
	// OS specific notes
	// - On Windows both SW and HW libraries may present
	// - On Linux only HW library only is available
	//   If more recent API features are needed, change the version accordingly
	mfxIMPL impl = options.values.impl;
	mfxVersion ver = { { 0, 1 } };
	//MFXVideoSession	cSession;

	sts = Initialize(impl, ver, &cSession, NULL);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
	return true;
}

inline mfxVideoParam CSdkEnc::EncodePrmInit( MFXVideoSession &cSession, CmdOptions options )
{
	// Set required video parameters for encode
	// - In this example we are encoding an AVC (H.264) stream
	mfxVideoParam mfxEncParams;
	memset(&mfxEncParams, 0, sizeof(mfxEncParams));

	mfxEncParams.mfx.CodecId				 = MFX_CODEC_AVC;
	mfxEncParams.mfx.TargetUsage			 = MFX_TARGETUSAGE_BALANCED;
	mfxEncParams.mfx.TargetKbps				 = options.values.Bitrate;
	mfxEncParams.mfx.RateControlMethod		 = MFX_RATECONTROL_VBR;
	mfxEncParams.mfx.FrameInfo.FrameRateExtN = options.values.FrameRateN;
	mfxEncParams.mfx.FrameInfo.FrameRateExtD = options.values.FrameRateD;
	mfxEncParams.mfx.FrameInfo.FourCC		 = MFX_FOURCC_NV12;

	mfxEncParams.mfx.FrameInfo.ChromaFormat	 = MFX_CHROMAFORMAT_YUV420;
	//mfxEncParams.mfx.FrameInfo.ChromaFormat	 = MFX_CHROMAFORMAT_YUV422;

	mfxEncParams.mfx.FrameInfo.PicStruct	 = MFX_PICSTRUCT_PROGRESSIVE;
	mfxEncParams.mfx.FrameInfo.CropX  		 = 0;
	mfxEncParams.mfx.FrameInfo.CropY		 = 0;
	mfxEncParams.mfx.FrameInfo.CropW		 = options.values.Width;
	mfxEncParams.mfx.FrameInfo.CropH		 = options.values.Height;
	mfxEncParams.mfx.GopRefDist				 = 1;
	mfxEncParams.mfx.NumRefFrame			 = 1;
	mfxEncParams.mfx.MaxDecFrameBuffering	 = 1;
	mfxEncParams.AsyncDepth					 = 1;
	// Width must be a multiple of 16
	// Height must be a multiple of 16 in case of frame picture and a multiple of 32 in case of field picture
	mfxEncParams.mfx.FrameInfo.Width = MSDK_ALIGN16(options.values.Width);
	mfxEncParams.mfx.FrameInfo.Height =
		(MFX_PICSTRUCT_PROGRESSIVE == mfxEncParams.mfx.FrameInfo.PicStruct) ?
		MSDK_ALIGN16(options.values.Height) :
		MSDK_ALIGN32(options.values.Height);

	mfxEncParams.IOPattern = MFX_IOPATTERN_IN_SYSTEM_MEMORY;
	return mfxEncParams;
}



inline	BOOL	CSdkEnc::MainEnc()
{
	CString strOutput;
	bool bEnableOutput = false;
	//画像ファイル読み込み
	cv::Mat InpMat = cv::imread("Start.jpg");

	//cv::Mat InpMat = cv::imread("Test.bmp");
	//cv::Mat TempYUV;
	//cv::cvtColor( InpMat, TempYUV,CV_RGB2YCrCb);

	cv::Mat TempYUV = RGB2YUV422( InpMat );
	//表色系変更
	//TempYUV = RGB2YUV422( InpMat );
	//cv::cvtColor( InpMat, TempYUV, CV_BGR2YUV);
	cv::imwrite( "TempYuv.bmp",TempYUV);
	mfxStatus sts = MFX_ERR_NONE;
	CmdOptions options;
	MFXVideoSession cSession;
	//cSessionの初期化，ファイルパラメータの入力
	PreparaStage1( TempYUV,  options, cSession, bEnableOutput);
	//Read output File(make)
	FILE* fSink = NULL;
	if (bEnableOutput) {
		MSDK_FOPEN(fSink, options.values.SinkName, "wb");
		MSDK_CHECK_POINTER(fSink, MFX_ERR_NULL_PTR);
	}
	/*
	//bool bEnableInput;  // if true, removes all YUV file reading (which is replaced by pre-initialized surface data). Workload runs for 1000 frames.
	bool bEnableOutput; // if true, removes all output bitsteam file writing and printing the progress
	CmdOptions options;

	 // =====================================================================
	// Intel Media SDK encode pipeline setup
	// - In this example we are encoding an AVC (H.264) stream
	// - For simplistic memory management, system memory surfaces are used to store the raw frames
	//   (Note that when using HW acceleration video surfaces are prefered, for better performance)
	//

	// Read options from the command line (if any is given)
	memset(&options, 0, sizeof(CmdOptions));
	options.ctx.options = OPTIONS_ENCODE;
	//options.ctx.usage = usage;							// よくわからん(使用方法の表示)?
	// Set default values:
	options.values.impl = MFX_IMPL_AUTO_ANY;

	// here we parse options
	ParseOptions(&options,TempYUV);
	
		if (!options.values.Width || !options.values.Height) {
		strOutput.Format("error: input video geometry not set (mandatory)\n");
		::OutputDebugString(strOutput);
		return FALSE;
	}
	if (!options.values.Bitrate) {
		strOutput.Format("error: bitrate not set (mandatory)\n");
		::OutputDebugString(strOutput);
		return FALSE;
	}
	if (!options.values.FrameRateN || !options.values.FrameRateD) {
		strOutput.Format("error: framerate not set (mandatory)\n");
		::OutputDebugString(strOutput);
		return FALSE;
	}
	//bEnableInput = (options.values.SourceName[0] != '\0');
	bEnableOutput = (options.values.SinkName[0] != '\0');

	 // Open input YV12 YUV file							// ファイルの読み込みスルー
	//FILE* fSource = NULL;
	//if (bEnableInput) {
	//	MSDK_FOPEN(fSource, options.values.SourceName, "rb");
	//	MSDK_CHECK_POINTER(fSource, MFX_ERR_NULL_PTR);
	//}

	// Create output elementary stream (ES) H.264 file	// ファイルの出力スルー
	FILE* fSink = NULL;
	if (bEnableOutput) {
		MSDK_FOPEN(fSink, options.values.SinkName, "wb");
		MSDK_CHECK_POINTER(fSink, MFX_ERR_NULL_PTR);
	}

	// Initialize Intel Media SDK session
	// - MFX_IMPL_AUTO_ANY selects HW acceleration if available (on any adapter)
	// - Version 1.0 is selected for greatest backwards compatibility.
	// OS specific notes
	// - On Windows both SW and HW libraries may present
	// - On Linux only HW library only is available
	//   If more recent API features are needed, change the version accordingly
	mfxIMPL impl = options.values.impl;
	mfxVersion ver = { { 0, 1 } };
	MFXVideoSession	cSession;

	sts = Initialize(impl, ver, &cSession, NULL);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
	*/
	// Create Media SDK encoder
	MFXVideoENCODE mfxENC(cSession);

	// Set required video parameters for encode
	// - In this example we are encoding an AVC (H.264) stream
	mfxVideoParam mfxEncParams = EncodePrmInit(cSession, options );
	/*
	memset(&mfxEncParams, 0, sizeof(mfxEncParams));

	mfxEncParams.mfx.CodecId				 = MFX_CODEC_AVC;
	mfxEncParams.mfx.TargetUsage			 = MFX_TARGETUSAGE_BALANCED;
	mfxEncParams.mfx.TargetKbps				 = options.values.Bitrate;
	mfxEncParams.mfx.RateControlMethod		 = MFX_RATECONTROL_VBR;
	mfxEncParams.mfx.FrameInfo.FrameRateExtN = options.values.FrameRateN;
	mfxEncParams.mfx.FrameInfo.FrameRateExtD = options.values.FrameRateD;
	mfxEncParams.mfx.FrameInfo.FourCC		 = MFX_FOURCC_NV12;

	//mfxEncParams.mfx.FrameInfo.ChromaFormat	 = MFX_CHROMAFORMAT_YUV420;
	mfxEncParams.mfx.FrameInfo.ChromaFormat	 = MFX_CHROMAFORMAT_YUV422;

	mfxEncParams.mfx.FrameInfo.PicStruct	 = MFX_PICSTRUCT_PROGRESSIVE;
	mfxEncParams.mfx.FrameInfo.CropX  		 = 0;
	mfxEncParams.mfx.FrameInfo.CropY		 = 0;
	mfxEncParams.mfx.FrameInfo.CropW		 = options.values.Width;
	mfxEncParams.mfx.FrameInfo.CropH		 = options.values.Height;
	mfxEncParams.mfx.GopRefDist				 = 1;
	mfxEncParams.mfx.NumRefFrame			 = 1;
	mfxEncParams.mfx.MaxDecFrameBuffering = 1;
	mfxEncParams.AsyncDepth = 1;
	// Width must be a multiple of 16
	// Height must be a multiple of 16 in case of frame picture and a multiple of 32 in case of field picture
	mfxEncParams.mfx.FrameInfo.Width = MSDK_ALIGN16(options.values.Width);
	mfxEncParams.mfx.FrameInfo.Height =
		(MFX_PICSTRUCT_PROGRESSIVE == mfxEncParams.mfx.FrameInfo.PicStruct) ?
		MSDK_ALIGN16(options.values.Height) :
		MSDK_ALIGN32(options.values.Height);

	mfxEncParams.IOPattern = MFX_IOPATTERN_IN_SYSTEM_MEMORY;
	*/
	// Validate video encode parameters (optional)
	// - In this example the validation result is written to same structure
	// - MFX_WRN_INCOMPATIBLE_VIDEO_PARAM is returned if some of the video parameters are not supported,
	//   instead the encoder will select suitable parameters closest matching the requested configuration
	sts = mfxENC.Query(&mfxEncParams, &mfxEncParams);
	MSDK_IGNORE_MFX_STS(sts, MFX_WRN_INCOMPATIBLE_VIDEO_PARAM);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

	// Query number of required surfaces for encoder
	mfxFrameAllocRequest EncRequest;
	memset(&EncRequest, 0, sizeof(EncRequest));
	sts = mfxENC.QueryIOSurf(&mfxEncParams, &EncRequest);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);


	//mfxU16 nEncSurfNum = EncRequest.NumFrameSuggested;	//どこでフレーム数の情報入れてるかわからないのでもしかしたらダミーファイルを用意してごまかすほうがいいかも
	mfxU16 nEncSurfNum = 1;

	// Allocate surfaces for encoder
	// - Width and height of buffer must be aligned, a multiple of 32
	// - Frame surface array keeps pointers all surface planes and general frame info
	mfxU16 width = (mfxU16) MSDK_ALIGN32(EncRequest.Info.Width);
	mfxU16 height = (mfxU16) MSDK_ALIGN32(EncRequest.Info.Height);
	mfxU8 bitsPerPixel = 12;        // NV12 format is a 12 bits per pixel format
	mfxU32 surfaceSize = width * height * bitsPerPixel / 8;
	mfxU8* surfaceBuffers = (mfxU8*) new mfxU8[surfaceSize * nEncSurfNum];

	// Allocate surface headers (mfxFrameSurface1) for encoder
	mfxFrameSurface1** pEncSurfaces = new mfxFrameSurface1 *[nEncSurfNum];
	MSDK_CHECK_POINTER(pEncSurfaces, MFX_ERR_MEMORY_ALLOC);
	for (int i = 0; i < nEncSurfNum; i++) {
		pEncSurfaces[i] = new mfxFrameSurface1;
		memset(pEncSurfaces[i], 0, sizeof(mfxFrameSurface1));
		memcpy(&(pEncSurfaces[i]->Info), &(mfxEncParams.mfx.FrameInfo), sizeof(mfxFrameInfo));	//エンコ用パラメータの中身をコピー
		pEncSurfaces[i]->Data.Y = &surfaceBuffers[surfaceSize * i];
		pEncSurfaces[i]->Data.U = pEncSurfaces[i]->Data.Y + width * height;
		pEncSurfaces[i]->Data.V = pEncSurfaces[i]->Data.U + 1;
		pEncSurfaces[i]->Data.Pitch = width;
		//if (!bEnableInput) {
		//	ClearYUVSurfaceSysMem(pEncSurfaces[i], width, height);
		//}
	}
	
	// Initialize the Media SDK encoder
	sts = mfxENC.Init(&mfxEncParams);
	MSDK_IGNORE_MFX_STS(sts, MFX_WRN_PARTIAL_ACCELERATION);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

	// Retrieve video parameters selected by encoder.
	// - BufferSizeInKB parameter is required to set bit stream buffer size
	mfxVideoParam par;
	memset(&par, 0, sizeof(par));
	sts = mfxENC.GetVideoParam(&par);
	//par.AsyncDepth = 1;
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

	// Prepare Media SDK bit stream buffer
	mfxBitstream mfxBS;
	memset(&mfxBS, 0, sizeof(mfxBS));
	mfxBS.MaxLength = par.mfx.BufferSizeInKB * 1000;
	mfxBS.Data = new mfxU8[mfxBS.MaxLength];
	MSDK_CHECK_POINTER(mfxBS.Data, MFX_ERR_MEMORY_ALLOC);

	//追加エンコードコントロール
	mfxEncodeCtrl EncodeCtrl;
	memset( &EncodeCtrl,0,sizeof( mfxEncodeCtrl ) );
	EncodeCtrl.FrameType = MFX_FRAMETYPE_I | MFX_FRAMETYPE_REF| MFX_FRAMETYPE_IDR;


	// ===================================
	// Start encoding the frames
	//

	mfxTime tStart, tEnd;
	mfxGetTime(&tStart);

	int nEncSurfIdx = 0;
	mfxSyncPoint syncp;
	mfxU32 nFrame = 0;

	//Matの更新を知らせるフラグ
	bool bNewMatflag = true;
	cv::Mat YUV;
	cv::Mat frame;
	//WebCamのおーぷん
	cv::VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH , 1280 );
	cap.set(CV_CAP_PROP_FRAME_HEIGHT , 960 );
	LARGE_INTEGER	liFreq, liStart, liEnd;
	memset( &liFreq, 0x00, sizeof liFreq );		liStart	= liEnd	= liFreq;

	if(cap.isOpened()){
	while(1)
	{
		::QueryPerformanceFrequency( &liFreq );

		::QueryPerformanceCounter( &liStart );
		cap >> frame;
		YUV = RGB2YUV422( frame);
		UV fUV;
		RGB2YUV420( frame, YUV,fUV );
		//cv::cvtColor(frame, YUV , CV_BGR2YCrCb);
		cv::imshow("raw",frame);
		//cv::imshow("YUV", YUV);
		int key = cv::waitKey(1);
		if(key == 113)	break;
	//
	// Stage 1: Main encoding loop
	//
	while (MFX_ERR_NONE <= sts || MFX_ERR_MORE_DATA == sts) {
		nEncSurfIdx = GetFreeSurfaceIndex(pEncSurfaces, nEncSurfNum);   // Find free frame surface
		MSDK_CHECK_ERROR(MFX_ERR_NOT_FOUND, nEncSurfIdx, MFX_ERR_MEMORY_ALLOC);
		//sts = LoadRawFrame(pEncSurfaces[nEncSurfIdx], fSource);
		if( bNewMatflag == false )
			continue;

		sts = LoadRawFrame(pEncSurfaces[nEncSurfIdx], YUV,fUV);

		MSDK_BREAK_ON_ERROR(sts);

		for (;;) {
			// Encode a frame asychronously (returns immediately)
			sts = mfxENC.EncodeFrameAsync( &EncodeCtrl , pEncSurfaces[nEncSurfIdx], &mfxBS, &syncp);
			//sts = mfxENC.EncodeFrameAsync(NULL, pEncSurfaces[nEncSurfIdx], &mfxBS, &syncp);


			if (MFX_ERR_NONE < sts && !syncp) {     // Repeat the call if warning and no output
				if (MFX_WRN_DEVICE_BUSY == sts)
					MSDK_SLEEP(1);  // Wait if device is busy, then repeat the same call
			} else if (MFX_ERR_NONE < sts && syncp) {
				sts = MFX_ERR_NONE;     // Ignore warnings if output is available
				break;
			} else if (MFX_ERR_NOT_ENOUGH_BUFFER == sts) {
				// Allocate more bitstream buffer memory here if needed...
				break;
			} else
				break;
		}

		if (MFX_ERR_NONE == sts) {
			sts = cSession.SyncOperation(syncp, 60000);      // Synchronize. Wait until encoded frame is ready
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			sts = WriteBitStreamFrame(&mfxBS, fSink);
			MSDK_BREAK_ON_ERROR(sts);


			++nFrame;
			if (bEnableOutput) {
				strOutput.Format("Frame number: %d\r", nFrame);
				::OutputDebugString(strOutput);
				fflush(stdout);
				if( nFrame > 60 )	break;//bNewMatflag = false;
			}
		}
	}

	::QueryPerformanceCounter( &liEnd );

	double	dProcTime	= (liEnd.QuadPart-liStart.QuadPart)*1000.0f/liFreq.QuadPart;


	CString	strOutput;
	strOutput.Format("%f ",dProcTime);
	
	//SetDlgItemText( IDC_ProcTime, strOutput );
	::OutputDebugString(strOutput);

	}
	}
	cv::destroyAllWindows();
	// MFX_ERR_MORE_DATA means that the input file has ended, need to go to buffering loop, exit in case of other errors
	MSDK_IGNORE_MFX_STS(sts, MFX_ERR_MORE_DATA);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
	/*
	//
	// Stage 1: Main encoding loop
	//
	while (MFX_ERR_NONE <= sts || MFX_ERR_MORE_DATA == sts) {
		nEncSurfIdx = GetFreeSurfaceIndex(pEncSurfaces, nEncSurfNum);   // Find free frame surface
		MSDK_CHECK_ERROR(MFX_ERR_NOT_FOUND, nEncSurfIdx, MFX_ERR_MEMORY_ALLOC);
		//sts = LoadRawFrame(pEncSurfaces[nEncSurfIdx], fSource);
		if( bNewMatflag == false )
			continue;

		sts = LoadRawFrame(pEncSurfaces[nEncSurfIdx], TempYUV);

		MSDK_BREAK_ON_ERROR(sts);

		for (;;) {
			// Encode a frame asychronously (returns immediately)
			sts = mfxENC.EncodeFrameAsync( &EncodeCtrl , pEncSurfaces[nEncSurfIdx], &mfxBS, &syncp);
			//sts = mfxENC.EncodeFrameAsync(NULL, pEncSurfaces[nEncSurfIdx], &mfxBS, &syncp);


			if (MFX_ERR_NONE < sts && !syncp) {     // Repeat the call if warning and no output
				if (MFX_WRN_DEVICE_BUSY == sts)
					MSDK_SLEEP(1);  // Wait if device is busy, then repeat the same call
			} else if (MFX_ERR_NONE < sts && syncp) {
				sts = MFX_ERR_NONE;     // Ignore warnings if output is available
				break;
			} else if (MFX_ERR_NOT_ENOUGH_BUFFER == sts) {
				// Allocate more bitstream buffer memory here if needed...
				break;
			} else
				break;
		}

		if (MFX_ERR_NONE == sts) {
			sts = cSession.SyncOperation(syncp, 60000);      // Synchronize. Wait until encoded frame is ready
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			sts = WriteBitStreamFrame(&mfxBS, fSink);
			MSDK_BREAK_ON_ERROR(sts);


			++nFrame;
			if (bEnableOutput) {
				strOutput.Format("Frame number: %d\r", nFrame);
				::OutputDebugString(strOutput);
				fflush(stdout);
				if( nFrame > 60 )	break;//bNewMatflag = false;
			}
		}
	}	
	// MFX_ERR_MORE_DATA means that the input file has ended, need to go to buffering loop, exit in case of other errors
	MSDK_IGNORE_MFX_STS(sts, MFX_ERR_MORE_DATA);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);
	*/


	//
	// Stage 2: Retrieve the buffered encoded frames
	//
	while (MFX_ERR_NONE <= sts) {
		for (;;) {
			// Encode a frame asychronously (returns immediately)
			sts = mfxENC.EncodeFrameAsync(NULL, NULL, &mfxBS, &syncp);

			if (MFX_ERR_NONE < sts && !syncp) {     // Repeat the call if warning and no output
				if (MFX_WRN_DEVICE_BUSY == sts)
					MSDK_SLEEP(1);  // Wait if device is busy, then repeat the same call
			} else if (MFX_ERR_NONE < sts && syncp) {
				sts = MFX_ERR_NONE;     // Ignore warnings if output is available
				break;
			} else
				break;
		}

		if (MFX_ERR_NONE == sts) {
			sts = cSession.SyncOperation(syncp, 60000);      // Synchronize. Wait until encoded frame is ready
			MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

			sts = WriteBitStreamFrame(&mfxBS, fSink);
			MSDK_BREAK_ON_ERROR(sts);

			++nFrame;
			if (bEnableOutput) {
				strOutput.Format("Frame number: %d\r", nFrame);
				::OutputDebugString(strOutput);
				fflush(stdout);
			}
		}
	}

	// MFX_ERR_MORE_DATA indicates that there are no more buffered frames, exit in case of other errors
	MSDK_IGNORE_MFX_STS(sts, MFX_ERR_MORE_DATA);
	MSDK_CHECK_RESULT(sts, MFX_ERR_NONE, sts);

	mfxGetTime(&tEnd);
	double elapsed = TimeDiffMsec(tEnd, tStart) / 1000;
	double fps = ((double)nFrame / elapsed);
	strOutput.Format("\nExecution time: %3.2f s (%3.2f fps)\n", elapsed, fps);
	::OutputDebugString(strOutput);
	
	
	// ===================================================================
	// Clean up resources
	//  - It is recommended to close Media SDK components first, before releasing allocated surfaces, since
	//    some surfaces may still be locked by internal Media SDK resources.

	mfxENC.Close();
	// session closed automatically on destruction

	for (int i = 0; i < nEncSurfNum; i++)
		delete pEncSurfaces[i];
	MSDK_SAFE_DELETE_ARRAY(pEncSurfaces);
	MSDK_SAFE_DELETE_ARRAY(mfxBS.Data);

	MSDK_SAFE_DELETE_ARRAY(surfaceBuffers);

	//if (fSource) fclose(fSource);
	if (fSink) fclose(fSink);

	Release();

	return TRUE;
}

//#endif	// _SDKENC_H_