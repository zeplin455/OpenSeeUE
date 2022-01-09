// Fill out your copyright notice in the Description page of Project Settings.


#include "OpenSeeComponent.h"
#include "Math/Quat.h"

// Sets default values for this component's properties
UOpenSeeComponent::UOpenSeeComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	Native = MakeShareable(new FUDPMessenger);
	LinkupCallbacks();

	// ...
}


// Called when the game starts
void UOpenSeeComponent::BeginPlay()
{
	Super::BeginPlay();
	Native->Settings = Settings;

	if (Settings.bShouldAutoOpenReceive)
	{
		OpenReceiveSocket(Settings.ReceiveIP, Settings.ReceivePort);
	}

	
}


// Called every frame
void UOpenSeeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

FQuat FOpenSeeTrackingData::LookRotation(FVector lookAt, FVector upDirection)
{
	FVector forward = lookAt;
	FVector up = upDirection;


	forward = forward.GetSafeNormal();
	up = up - (forward * FVector::DotProduct(up, forward));
	up = up.GetSafeNormal();

	///////////////////////


	FVector vector = forward.GetSafeNormal();
	FVector vector2 = FVector::CrossProduct(up, vector);
	FVector vector3 = FVector::CrossProduct(vector, vector2);
	float m00 = vector.X;
	float m01 = vector.Y;
	float m02 = vector.Z;
	float m10 = vector2.X;
	float m11 = vector2.Y;
	float m12 = vector2.Z;
	float m20 = vector3.X;
	float m21 = vector3.Y;
	float m22 = vector3.Z;

	float num8 = (m00 + m11) + m22;
	FQuat quaternion = FQuat();

	if (num8 > 0.0f)
	{
		float num = (float)FMath::Sqrt(num8 + 1.0f);
		quaternion.W = num * 0.5f;
		num = 0.5f / num;
		quaternion.X = (m12 - m21) * num;
		quaternion.Y = (m20 - m02) * num;
		quaternion.Z = (m01 - m10) * num;
		return quaternion;
	}

	if ((m00 >= m11) && (m00 >= m22))
	{
		float num7 = (float)FMath::Sqrt(((1.0f + m00) - m11) - m22);
		float num4 = 0.5f / num7;
		quaternion.X = 0.5f * num7;
		quaternion.Y = (m01 + m10) * num4;
		quaternion.Z = (m02 + m20) * num4;
		quaternion.W = (m12 - m21) * num4;
		return quaternion;
	}

	if (m11 > m22)
	{
		float num6 = (float)FMath::Sqrt(((1.0f + m11) - m00) - m22);
		float num3 = 0.5f / num6;
		quaternion.X = (m10 + m01) * num3;
		quaternion.Y = 0.5f * num6;
		quaternion.Z = (m21 + m12) * num3;
		quaternion.W = (m20 - m02) * num3;
		return quaternion;
	}

	float num5 = (float)FMath::Sqrt(((1.0f + m22) - m00) - m11);
	float num2 = 0.5f / num5;
	quaternion.X = (m20 + m02) * num2;
	quaternion.Y = (m21 + m12) * num2;
	quaternion.Z = 0.5f * num5;
	quaternion.W = (m01 - m10) * num2;


	return quaternion;
}

float FOpenSeeTrackingData::readFloat(uint8_t* bytes, int& offset) {
	float f;
	uint8_t* f_ptr = (uint8_t*)&f;

	f_ptr[3] = bytes[3 + offset];
	f_ptr[2] = bytes[2 + offset];
	f_ptr[1] = bytes[1 + offset];
	f_ptr[0] = bytes[0 + offset];

	offset += 4;
	return f;
}

double FOpenSeeTrackingData::readDouble(uint8_t* bytes, int& offset) {
	double f;

	uint8_t* f_ptr = (uint8_t*)&f;

	f_ptr[7] = bytes[7 + offset];
	f_ptr[6] = bytes[6 + offset];
	f_ptr[5] = bytes[5 + offset];
	f_ptr[4] = bytes[4 + offset];
	f_ptr[3] = bytes[3 + offset];
	f_ptr[2] = bytes[2 + offset];
	f_ptr[1] = bytes[1 + offset];
	f_ptr[0] = bytes[0 + offset];

	offset += 8;

	return f;
}

int FOpenSeeTrackingData::readInt(uint8_t* bytes, int& offset) {
	int i;
	uint8_t* f_ptr = (uint8_t*)&i;

	f_ptr[3] = bytes[3 + offset];
	f_ptr[2] = bytes[2 + offset];
	f_ptr[1] = bytes[1 + offset];
	f_ptr[0] = bytes[0 + offset];

	offset += 4;

	return i;
}

FVector2D FOpenSeeTrackingData::readVector2(uint8_t* bytes, int& offset)
{
	FVector2D vec2d;
	vec2d.X = readFloat(bytes, offset);
	vec2d.Y = readFloat(bytes, offset);
	return vec2d;
}

FVector FOpenSeeTrackingData::readVector3(uint8_t* bytes, int& offset)
{
	FVector vec3d;
	vec3d.X = readFloat(bytes, offset);
	vec3d.Y = -readFloat(bytes, offset);
	vec3d.Z = readFloat(bytes, offset);
	return vec3d;
}

FVector FOpenSeeTrackingData::swapX(FVector v)
{
	v.X = -v.X;
	return v;
}

FQuat FOpenSeeTrackingData::readQuaternion(uint8_t* bytes, int& offset) {
	float x = readFloat(bytes, offset);
	float y = readFloat(bytes, offset);
	float z = readFloat(bytes, offset);
	float w = readFloat(bytes, offset);
	FQuat q = FQuat(x, y, z, w);
	return q;
}

void FOpenSeeTrackingData::readFromPacket(uint8_t* b, int o) {
	time = readDouble(b, o);
	id = readInt(b, o);

	cameraResolution = readVector2(b, o);
	rightEyeOpen = readFloat(b, o);
	leftEyeOpen = readFloat(b, o);

	char got3D = b[o];
	o++;
	got3DPoints = false;
	if (got3D != 0)
		got3DPoints = true;

	fit3DError = readFloat(b, o);
	rawQuaternion = readQuaternion(b, o);
	FQuat convertedQuaternion = FQuat(-rawQuaternion.X, rawQuaternion.Y, -rawQuaternion.Z, rawQuaternion.W);
	rawEuler = readVector3(b, o);

	rotation = rawEuler;
	rotation.Z = fmod((rotation.Z - 90), 360);
	rotation.X = fmod(-(rotation.X + 180), 360);

	float x = readFloat(b, o);
	float y = readFloat(b, o);
	float z = readFloat(b, o);
	translation = FVector(-y, x, -z);

	confidence.SetNum(nPoints);
	for (int i = 0; i < nPoints; i++) {
		confidence[i] = readFloat(b, o);
	}

	points.SetNum(nPoints);
	for (int i = 0; i < nPoints; i++) {
		points[i] = readVector2(b, o);
	}

	points3D.SetNum(nPoints + 2);
	for (int i = 0; i < nPoints + 2; i++) {
		points3D[i] = readVector3(b, o);
	}
	float Rads180 = FMath::DegreesToRadians(180);
	rightGaze = LookRotation(swapX(points3D[66]) - swapX(points3D[68]), FVector::UpVector) * FQuat(FVector::RightVector, Rads180) * FQuat(FVector::ForwardVector, Rads180);
	leftGaze = LookRotation(swapX(points3D[67]) - swapX(points3D[69]), FVector::UpVector) * FQuat(FVector::RightVector, Rads180) * FQuat(FVector::ForwardVector, Rads180);

	//features = OpenSeeFeatures();
	features.EyeLeft = readFloat(b, o);
	features.EyeRight = readFloat(b, o);
	features.EyebrowSteepnessLeft = readFloat(b, o);
	features.EyebrowUpDownLeft = readFloat(b, o);
	features.EyebrowQuirkLeft = readFloat(b, o);
	features.EyebrowSteepnessRight = readFloat(b, o);
	features.EyebrowUpDownRight = readFloat(b, o);
	features.EyebrowQuirkRight = readFloat(b, o);
	features.MouthCornerUpDownLeft = readFloat(b, o);
	features.MouthCornerInOutLeft = readFloat(b, o);
	features.MouthCornerUpDownRight = readFloat(b, o);
	features.MouthCornerInOutRight = readFloat(b, o);
	features.MouthOpen = readFloat(b, o);
	features.MouthWide = readFloat(b, o);
}

static FQuat NormalizeQuaternion(float x, float y, float z, float w) {

	float lengthD = 1.0f / (w * w + x * x + y * y + z * z);
	w *= lengthD;
	x *= lengthD;
	y *= lengthD;
	z *= lengthD;

	return FQuat(x, y, z, w);
}

//Changes the sign of the quaternion components. This is not the same as the inverse.
static FQuat InverseSignQuaternion(FQuat q) {

	return FQuat(-q.X, -q.Y, -q.Z, -q.W);
}

bool AreQuaternionsClose(FQuat q1, FQuat q2) {

	float dot = q1 | q2;

	if (dot < 0.0f) {

		return false;
	}

	else {

		return true;
	}
}

FQuat AverageQuaternion(FVector4& cumulative, FQuat newRotation, FQuat firstRotation, int addAmount) {

	float w = 0.0f;
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

	//Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
	//q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
	if (!AreQuaternionsClose(newRotation, firstRotation)) {

		newRotation = InverseSignQuaternion(newRotation);
	}

	//Average the values
	float addDet = 1.f / (float)addAmount;
	cumulative.W += newRotation.W;
	w = cumulative.W * addDet;
	cumulative.X += newRotation.X;
	x = cumulative.X * addDet;
	cumulative.Y += newRotation.Y;
	y = cumulative.Y * addDet;
	cumulative.Z += newRotation.Z;
	z = cumulative.Z * addDet;

	//note: if speed is an issue, you can skip the normalization step
	return NormalizeQuaternion(x, y, z, w);
}



void UOpenSeeComponent::LinkupCallbacks()
{
	Native->OnReceiveOpened = [this](int32 Port)
	{
		Settings.ReceiveIP = Native->Settings.ReceiveIP;
		Settings.ReceivePort = Native->Settings.ReceivePort;

		OnReceiveSocketOpened.Broadcast(Port);
	};
	Native->OnReceiveClosed = [this](int32 Port)
	{
		OnReceiveSocketClosed.Broadcast(Port);
	};
	Native->OnReceivedData = [this](const FOpenSeeTrackingData& Data, const FString& Endpoint)
	{
		OnReceivedTrackingInfo.Broadcast(Data, Endpoint);
	};
}

bool UOpenSeeComponent::CloseReceiveSocket()
{
	return Native->CloseReceiveSocket();
}


bool UOpenSeeComponent::OpenReceiveSocket(const FString& InListenIp /*= TEXT("0.0.0.0")*/, const int32 InListenPort /*= 3002*/)
{
	//Sync side effect sampled settings
	Native->Settings.bShouldAutoOpenReceive = Settings.bShouldAutoOpenReceive;
	Native->Settings.ReceiveSocketName = Settings.ReceiveSocketName;
	Native->Settings.BufferSize = Settings.BufferSize;

	return Native->OpenReceiveSocket(InListenIp, InListenPort);
}



void UOpenSeeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	CloseReceiveSocket();

	Native->ClearReceiveCallbacks();

	Super::EndPlay(EndPlayReason);
}

FUDPMessenger::FUDPMessenger()
{
	SenderSocket = nullptr;
	ReceiverSocket = nullptr;

	ClearReceiveCallbacks();
}

FUDPMessenger::~FUDPMessenger()
{
	CloseReceiveSocket();
	ClearReceiveCallbacks();
}


bool FUDPMessenger::OpenReceiveSocket(const FString& InListenIP /*= TEXT("0.0.0.0")*/, const int32 InListenPort /*= 3002*/)
{
	//Sync and overwrite settings
	if (Settings.bShouldOpenReceiveToBoundSendPort)
	{
		if (Settings.SendBoundPort == 0)
		{
			UE_LOG(LogTemp, Error, TEXT("FUDPNative::OpenReceiveSocket Can't bind to SendBoundPort if send socket hasn't been opened before this call."));
			return false;
		}
		Settings.ReceiveIP = Settings.SendIP;
		Settings.ReceivePort = Settings.SendBoundPort;
	}
	else
	{
		Settings.ReceiveIP = InListenIP;
		Settings.ReceivePort = InListenPort;
	}

	bool bDidOpenCorrectly = true;
	bDidOpenCorrectly = CloseReceiveSocket();

	FIPv4Address Addr;
	FIPv4Address::Parse(Settings.ReceiveIP, Addr);

	//Create Socket
	FIPv4Endpoint Endpoint(Addr, Settings.ReceivePort);

	ReceiverSocket = FUdpSocketBuilder(*Settings.ReceiveSocketName)
		.AsNonBlocking()
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(Settings.BufferSize);

	FTimespan ThreadWaitTime = FTimespan::FromMilliseconds(100);
	FString ThreadName = FString::Printf(TEXT("UDP RECEIVER-FUDPNative"));
	UDPReceiver = new FUdpSocketReceiver(ReceiverSocket, ThreadWaitTime, *ThreadName);

	UDPReceiver->OnDataReceived().BindLambda([this](const FArrayReaderPtr& DataPtr, const FIPv4Endpoint& Endpoint)
		{
			if (!OnReceivedData)
			{
				return;
			}

			TArray<uint8> Data;
			Data.AddUninitialized(DataPtr->TotalSize());
			DataPtr->Serialize(Data.GetData(), DataPtr->TotalSize());

			FString SenderIp = Endpoint.Address.ToString();
			FOpenSeeTrackingData openSee;
			openSee.readFromPacket(Data.GetData(), 0);

			if (Settings.bReceiveDataOnGameThread)
			{
				//Copy data to receiving thread via lambda capture
				AsyncTask(ENamedThreads::GameThread, [this, openSee, SenderIp]()
					{
						//double check we're still bound on this thread
						if (OnReceivedData)
						{
							OnReceivedData(openSee, SenderIp);
						}
					});
			}
			else
			{
				OnReceivedData(openSee, SenderIp);
			}
		});

	if (OnReceiveOpened)
	{
		OnReceiveOpened(Settings.ReceivePort);
	}

	UDPReceiver->Start();

	return bDidOpenCorrectly;
}

bool FUDPMessenger::CloseReceiveSocket()
{
	bool bDidCloseCorrectly = true;

	if (ReceiverSocket)
	{
		UDPReceiver->Stop();
		delete UDPReceiver;
		UDPReceiver = nullptr;

		bDidCloseCorrectly = ReceiverSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ReceiverSocket);
		ReceiverSocket = nullptr;

		if (OnReceiveClosed)
		{
			OnReceiveClosed(Settings.ReceivePort);
		}
	}

	return bDidCloseCorrectly;
}

void FUDPMessenger::ClearReceiveCallbacks()
{
	OnReceivedData = nullptr;
	OnReceiveOpened = nullptr;
	OnReceiveClosed = nullptr;
}

FUDPNetworkSettings::FUDPNetworkSettings()
{
	bShouldAutoOpenSend = true;
	bShouldAutoOpenReceive = true;
	bShouldOpenReceiveToBoundSendPort = false;
	bReceiveDataOnGameThread = true;
	ReceiveIP = FString(TEXT("127.0.0.1"));
	ReceivePort = 11573;
	SendSocketName = FString(TEXT("ue4-dgram-send"));
	ReceiveSocketName = FString(TEXT("ue4-dgram-receive"));

	BufferSize = 2 * 1024 * 1024;	//default roughly 2mb
}